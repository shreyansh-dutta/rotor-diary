# Rotor Diary

Static build journal for a student-built Arduino quadcopter project, plus the corrected Arduino flight-control sketch.

## Project Files

- `index.html` - static project journal for the drone build.
- `Drone.ino` - Arduino sketch with FS-R6B receiver input, MPU6050 sensing, PID stabilization, ESC output, and failsafe checks.

## Current Wiring Summary

- Frame: F450 X-frame quadcopter frame
- Motors: 4x 1000KV brushless motors
- ESC signals: `D4`, `D5`, `D6`, `D7`
- Receiver: FS-R6B active channels `D8`, `D9`, `D10`, `D11`, `D12`; CH6 is optional/not used
- Transmitter/controller: FS-CT6B
- Kill switch: Switch B forces CH3 on `D10` to about `1008-1012us`
- Throttle/kill logic: CH3 `<= 1040us` is disarmed; CH3 `> 1040us` allows motors only after throttle cut was seen once
- Receiver failsafe: CH3 timeout is `1,000,000us` / 1 second
- MPU6050: SDA `A4`, SCL `A5`
- ESC idle: `1150us`; ESC cap: `1850us`
- Assembly: motor, ESC, signal, and power wiring soldered by hand
- Common ground is required across Arduino, receiver, MPU6050, and ESC signal grounds.

## Local Preview

From this folder, serve the site with a simple static server and open the printed local URL.

```bash
python3 -m http.server 5173
```

Then visit `http://localhost:5173`.

## Publishing Note

This repository stores the project files and proof images. It does not need GitHub Pages enabled unless I choose to publish it there later.

## Before Sharing Publicly

- Add CAD screenshots, wiring diagram exports, and test videos if they are available.
- Keep future journal updates in `index.html` so every admissions reader sees the same content.
- Test with props removed first before any motor or hover test.
