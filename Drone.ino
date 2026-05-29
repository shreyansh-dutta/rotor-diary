#include <Wire.h>

// Homemade Arduino Uno quadcopter controller.
// Motor pins are intentionally D4-D7 so all ESC pulses can be written on PORTD.

#define MPU_ADDR 0x68

#define LOOP_US 4000UL
#define ESC_STOP 1000
#define ESC_IDLE 1150
#define ESC_MAX 1850

#define ARM_THRESHOLD_US 1100
#define MIN_VALID_RX_US 900
#define MAX_VALID_RX_US 2200
#define RX_FAILSAFE_US 120000UL

// Receiver channel indexes.
enum {
  CH_ROLL = 0,
  CH_PITCH = 1,
  CH_THROTTLE = 2,
  CH_YAW = 3,
  CH_AUX = 4,
  CH_ARM = 5,
  RX_CHANNELS = 6
};

// PID axis indexes.
enum {
  AXIS_PITCH = 0,
  AXIS_ROLL = 1,
  AXIS_YAW = 2
};

volatile uint32_t rx_timer[RX_CHANNELS];
volatile uint32_t rx_last_seen[RX_CHANNELS];
volatile uint8_t last_pinb;
volatile uint8_t last_pinc;
volatile int receiver_input[RX_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1190};

float gyro_offset[3];
float gyro_raw[3];
float acc_raw[3];
float acc_angle[2];
float angle_pitch;
float angle_roll;

float pid_output[3];
float pid_i_mem[3];
float pid_last_error[3];

const float Kp = 1.3;
const float Ki = 0.04;
const float Kd = 22.0;
const float Kp_level = 4.5;

const float pitch_level_offset = 1.5;
const float roll_level_offset = -2.1;

int esc[4] = {ESC_STOP, ESC_STOP, ESC_STOP, ESC_STOP};
int throttle;
bool armed = false;
bool first_angle = true;
uint32_t loop_timer;

static bool valid_rx_pulse(uint16_t pulse) {
  return pulse >= MIN_VALID_RX_US && pulse <= MAX_VALID_RX_US;
}

static int rx_read(uint8_t channel) {
  uint8_t old_sreg = SREG;
  noInterrupts();
  int value = receiver_input[channel];
  SREG = old_sreg;
  return value;
}

static bool receiver_is_alive() {
  uint32_t now = micros();
  for (uint8_t i = 0; i < RX_CHANNELS; i++) {
    uint32_t last_seen;
    uint8_t old_sreg = SREG;
    noInterrupts();
    last_seen = rx_last_seen[i];
    SREG = old_sreg;

    if (last_seen == 0 || now - last_seen > RX_FAILSAFE_US) {
      return false;
    }
  }
  return true;
}

void setup() {
  DDRD |= B11110000;    // D4, D5, D6, D7 are ESC signal outputs.
  PORTD &= B00001111;   // Keep all motor outputs low.

  pinMode(8, INPUT);    // CH1 roll
  pinMode(9, INPUT);    // CH2 pitch
  pinMode(10, INPUT);   // CH3 throttle
  pinMode(11, INPUT);   // CH4 yaw
  pinMode(A0, INPUT);   // CH5 aux
  pinMode(A1, INPUT);   // CH6 arming switch

  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;            // 400 kHz I2C on a 16 MHz Uno.

  setup_mpu();

  Serial.println(F("Calibrating gyro. Keep the drone perfectly still."));
  for (int i = 0; i < 2000; i++) {
    read_mpu();
    gyro_offset[0] += gyro_raw[0];
    gyro_offset[1] += gyro_raw[1];
    gyro_offset[2] += gyro_raw[2];

    PORTD |= B11110000;
    delayMicroseconds(ESC_STOP);
    PORTD &= B00001111;
    delay(2);
  }
  gyro_offset[0] /= 2000.0;
  gyro_offset[1] /= 2000.0;
  gyro_offset[2] /= 2000.0;

  last_pinb = PINB;
  last_pinc = PINC;

  PCICR |= (1 << PCIE0) | (1 << PCIE1);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3); // D8-D11
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);                                  // A0-A1

  Serial.println(F("Ready. Props off for bench testing."));
  loop_timer = micros();
}

void loop() {
  read_mpu();

  gyro_raw[0] -= gyro_offset[0];
  gyro_raw[1] -= gyro_offset[1];
  gyro_raw[2] -= gyro_offset[2];

  calculate_angles();
  update_arming();

  throttle = rx_read(CH_THROTTLE);
  if (!valid_rx_pulse(throttle)) {
    throttle = ESC_STOP;
  }

  if (armed && throttle > 1050 && receiver_is_alive()) {
    calculate_pid();

    // Motor order:
    // esc[0] D4 front-left CW, esc[1] D5 front-right CCW,
    // esc[2] D6 rear-left CCW, esc[3] D7 rear-right CW.
    esc[0] = throttle + pid_output[AXIS_ROLL] - pid_output[AXIS_PITCH] + pid_output[AXIS_YAW];
    esc[1] = throttle - pid_output[AXIS_ROLL] - pid_output[AXIS_PITCH] - pid_output[AXIS_YAW];
    esc[2] = throttle + pid_output[AXIS_ROLL] + pid_output[AXIS_PITCH] - pid_output[AXIS_YAW];
    esc[3] = throttle - pid_output[AXIS_ROLL] + pid_output[AXIS_PITCH] + pid_output[AXIS_YAW];

    for (uint8_t i = 0; i < 4; i++) {
      esc[i] = constrain(esc[i], ESC_IDLE, ESC_MAX);
    }
  } else {
    stop_motors_and_reset_pid();
  }

  while (micros() - loop_timer < LOOP_US) {
    // Fixed 250 Hz control loop.
  }
  loop_timer = micros();
  write_esc_pulses();
}

void setup_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);     // PWR_MGMT_1
  Wire.write(0x00);     // Wake up.
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);     // GYRO_CONFIG
  Wire.write(0x08);     // +/- 500 deg/s, scale factor 65.5 LSB/(deg/s).
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);     // ACCEL_CONFIG
  Wire.write(0x10);     // +/- 8 g.
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);     // CONFIG
  Wire.write(0x03);     // DLPF around 44 Hz gyro / 42 Hz accel.
  Wire.endTransmission();
}

void read_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14) {
    acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read();
    Wire.read();
    gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
  }
}

void calculate_angles() {
  angle_pitch += gyro_raw[AXIS_PITCH] * 0.0000611; // 1 / 65.5 / 250 Hz
  angle_roll += gyro_raw[AXIS_ROLL] * 0.0000611;

  angle_pitch += angle_roll * sin(gyro_raw[AXIS_YAW] * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_raw[AXIS_YAW] * 0.000001066);

  float acc_total_vector = sqrt(
    acc_raw[0] * acc_raw[0] +
    acc_raw[1] * acc_raw[1] +
    acc_raw[2] * acc_raw[2]
  );

  if (acc_total_vector > 0.0) {
    if (abs(acc_raw[1]) < acc_total_vector) {
      acc_angle[AXIS_PITCH] = asin((float)acc_raw[1] / acc_total_vector) * 57.296 + pitch_level_offset;
    }
    if (abs(acc_raw[0]) < acc_total_vector) {
      acc_angle[AXIS_ROLL] = asin((float)acc_raw[0] / acc_total_vector) * -57.296 + roll_level_offset;
    }
  }

  if (first_angle) {
    angle_pitch = acc_angle[AXIS_PITCH];
    angle_roll = acc_angle[AXIS_ROLL];
    first_angle = false;
  } else {
    angle_pitch = angle_pitch * 0.999 + acc_angle[AXIS_PITCH] * 0.001;
    angle_roll = angle_roll * 0.999 + acc_angle[AXIS_ROLL] * 0.001;
  }
}

void update_arming() {
  int arm_channel = rx_read(CH_ARM);

  // FS-CT6B Switch B on this build: toward you is about 1012 us, away is about 1190 us.
  armed = valid_rx_pulse(arm_channel) && arm_channel < ARM_THRESHOLD_US && receiver_is_alive();

  if (!armed) {
    stop_motors_and_reset_pid();
  }
}

void stop_motors_and_reset_pid() {
  for (uint8_t i = 0; i < 4; i++) {
    esc[i] = ESC_STOP;
  }
  for (uint8_t i = 0; i < 3; i++) {
    pid_i_mem[i] = 0;
    pid_last_error[i] = 0;
    pid_output[i] = 0;
  }
}

void calculate_pid() {
  float level_pitch = angle_pitch * Kp_level;
  float level_roll = angle_roll * Kp_level;

  for (uint8_t i = 0; i < 3; i++) {
    float setpoint = 0.0;

    if (i == AXIS_PITCH) {
      setpoint = (rx_read(CH_PITCH) - 1500) / 3.0 - level_pitch;
    } else if (i == AXIS_ROLL) {
      setpoint = (rx_read(CH_ROLL) - 1500) / 3.0 - level_roll;
    } else {
      setpoint = (rx_read(CH_YAW) - 1500) / 3.0;
    }

    float measured_rate = gyro_raw[i] / 65.5;
    float error = measured_rate - setpoint;
    pid_i_mem[i] += Ki * error;
    pid_i_mem[i] = constrain(pid_i_mem[i], -400, 400);
    pid_output[i] = Kp * error + pid_i_mem[i] + Kd * (error - pid_last_error[i]);
    pid_output[i] = constrain(pid_output[i], -400, 400);
    pid_last_error[i] = error;
  }
}

void write_esc_pulses() {
  PORTD |= B11110000;
  uint32_t pulse_start = micros();

  while (PORTD & B11110000) {
    uint32_t diff = micros() - pulse_start;
    if (esc[0] <= diff) PORTD &= B11101111; // D4
    if (esc[1] <= diff) PORTD &= B11011111; // D5
    if (esc[2] <= diff) PORTD &= B10111111; // D6
    if (esc[3] <= diff) PORTD &= B01111111; // D7
    if (diff > 2050) {
      PORTD &= B00001111;
      break;
    }
  }
}

void handle_pin_change(uint8_t channel, bool is_high, uint32_t now) {
  if (is_high) {
    rx_timer[channel] = now;
  } else {
    uint16_t pulse = now - rx_timer[channel];
    if (valid_rx_pulse(pulse)) {
      receiver_input[channel] = pulse;
      rx_last_seen[channel] = now;
    }
  }
}

ISR(PCINT0_vect) {
  uint32_t now = micros();
  uint8_t changed = PINB ^ last_pinb;
  uint8_t pins = PINB;
  last_pinb = pins;

  if (changed & (1 << 0)) handle_pin_change(CH_ROLL, pins & (1 << 0), now);     // D8
  if (changed & (1 << 1)) handle_pin_change(CH_PITCH, pins & (1 << 1), now);    // D9
  if (changed & (1 << 2)) handle_pin_change(CH_THROTTLE, pins & (1 << 2), now); // D10
  if (changed & (1 << 3)) handle_pin_change(CH_YAW, pins & (1 << 3), now);      // D11
}

ISR(PCINT1_vect) {
  uint32_t now = micros();
  uint8_t changed = PINC ^ last_pinc;
  uint8_t pins = PINC;
  last_pinc = pins;

  if (changed & (1 << 0)) handle_pin_change(CH_AUX, pins & (1 << 0), now);      // A0
  if (changed & (1 << 1)) handle_pin_change(CH_ARM, pins & (1 << 1), now);      // A1
}
