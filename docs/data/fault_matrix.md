| Fault | Runs | Detected | Detection time | Detected by | Fallback | Error during fault (mean / max) | Recovered |
|---|---|---|---|---|---|---|---|
| baseline, no fault | 5 | 0/5 | — | — | none | 0.05 / 0.17 m | 5/5 |
| GPS receiver stops | 5 | 5/5 | 4.5 s (4.4–4.6) | Auto switched to Source 2 | switched to optical flow | 0.32 / 0.92 m | 5/5 |
| GPS position jumps 5 m north, fix stays good | 5 | 5/5 | 0.2 s (0.1–0.3) | SP > 1 | rejected GPS | 4.18 / 5.16 m | 5/5 |
| IMU 1 accelerometer X bias 0.5 m/s^2 | 5 | 0/5 | — | — | none | 0.23 / 0.78 m | 5/5 |
| IMU 1 gyro X bias 0.02 rad/s | 5 | 0/5 | — | — | none | 0.28 / 0.89 m | 5/5 |
| IMU 1 accelerometer X bias 2.0 m/s^2 | 5 | 5/5 | 1.4 s (1.3–1.5) | EKF3 lane switch 1 | lane switch | 0.06 / 0.88 m | 5/5 |
| IMU 1 gyro X bias 0.1 rad/s | 5 | 5/5 | 2.5 s (2.4–2.6) | EKF3 lane switch 1 | lane switch | 0.06 / 1.00 m | 3/5 |
| compass 1 X offset set to 200 mGauss | 5 | 5/5 | 0.2 s (0.2–0.3) | SM > 1 | compass switch | 0.05 / 0.18 m | 5/5 |
| compass 1 stops | 5 | 5/5 | 10.0 s (10.0–10.1) | EKF3 IMU0 switching to compass 1 | compass switch | 0.05 / 0.19 m | 5/5 |
