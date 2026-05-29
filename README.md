# Rotor Diary

Static build journal for a student-built Arduino quadcopter project, plus the corrected Arduino flight-control sketch.

## Project Files

- `index.html` - static project journal for the drone build.
- `Drone.ino` - Arduino sketch with 6-channel receiver input, MPU6050 sensing, PID stabilization, ESC output, and failsafe checks.

## Current Wiring Summary

- Frame: F450 quadcopter frame
- Motors: 4x 850KV brushless motors
- ESC signals: `D4`, `D5`, `D6`, `D7`
- Receiver channels: `D8`, `D9`, `D10`, `D11`, `A0`, `A1`
- Arming switch: CH6 on `A1`, armed below `1100us`
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

## GitHub Pages Hosting

1. Create a GitHub repository for the project.
2. Add `index.html`, `Drone.ino`, and this `README.md` at the repository root.
3. Push the files to the `main` branch.
4. In GitHub, open **Settings -> Pages**.
5. Set **Source** to **Deploy from a branch**.
6. Set **Branch** to `main` and folder to `/root`.
7. Save, wait for GitHub Pages to publish, then use the generated public URL in the college application.

## Before Sharing Publicly

- Add CAD screenshots, wiring diagram exports, and test videos if they are available.
- Keep future journal updates in `index.html` so every admissions reader sees the same content.
- Test with props removed first before any motor or hover test.
