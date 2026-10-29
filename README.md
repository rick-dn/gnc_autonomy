# GNC Autonomy

Guidance, navigation and control experiments on ArduPilot, run in SITL with
Gazebo Harmonic and ROS 2 (MAVROS). Each experiment injects a fault or swaps a
sensor and measures what the vehicle actually does against ground truth.

## Experiments

### 1. GPS loss → optical flow fallback

[![Screen recording](https://img.youtube.com/vi/muZEITKe7_o/hqdefault.jpg)](https://youtu.be/muZEITKe7_o)

GPS is switched off mid-hover. A ROS 2 node notices and moves EKF3 over to
optical flow, scaled by a downward rangefinder, and the quad keeps flying.

- Switched to flow 1.3 s after GPS was lost
- Held a hover for ~56 s with 0.52 m mean position error
- Real drift was 0.87 m while the EKF thought it had moved 3 cm

Details, method and plots: [docs/p1_gps_loss_flow_fallback.md](docs/p1_gps_loss_flow_fallback.md)

### 2. EKF3 fault matrix

Nine fault cases, five scripted runs each, on SITL's built-in physics with no
Gazebo or ROS: GPS loss and glitch, accelerometer and gyro bias, compass
offset and failure. Each run is scored for detection time, what the vehicle
did, and position error against ground truth.

- A 5 m GPS glitch is caught in 0.2 s but only refused for 10 s, then EKF3
  accepts it and stays 5 m off
- Small IMU biases go undetected; large ones trigger a lane switch within
  1.5–2.5 s with almost no position error
- ArduPilot's stock GPS/flow switching script misses a total GPS loss; fixed
  here with a 3D-fix check

Details, method and plots: [docs/fault_matrix.md](docs/fault_matrix.md)

### 3. Custom INS/GNSS estimator vs EKF3

A 15-state error-state EKF written from scratch in Python, run offline on
SITL flight logs (400 Hz IMU, GPS, baro) and compared with EKF3 against
ground truth.

- Position within a few cm of truth, same as EKF3
- Roll and pitch match EKF3 (0.10° / 0.07° vs 0.10° / 0.08°); yaw is weaker
  since it only uses the compass at startup
- With an innovation gate it holds off the 5 m GPS glitch for the full
  minute (0.15 m error) where EKF3 gives in after 10 s

Details, method and plots: [docs/estimator.md](docs/estimator.md)

### Next

- External nav from ground-truth pose, with noise, delay and source switching
- C2 link-loss failsafes

## Layout

```
gnc_autonomy/
├── scripts/                # Gazebo launchers (start_rangfinder.sh, start_estimator.sh)
├── ga_sim/
│   ├── params/             # ArduPilot and MAVROS param files
│   └── scripts/            # onboard Lua (GPS/optical flow source switching)
├── ga_faults/              # batch fault runner, fault matrix, plots
├── ga_est/                 # custom estimator: log loader, ESKF, comparisons
├── ga_ws/src/
│   ├── ga_vehicle/         # MAVROS adapter, source_switcher, shuttle_flight
│   ├── ga_extnav_bridge/   # pose source -> external nav (planned)
│   └── ga_bringup/         # launch files (planned)
├── tools/eval/             # log analysis and plots for experiment 1
└── docs/                   # write-ups, figures and result tables
```

The Gazebo model (`iris_with_rangefinder`) and world
(`iris_warehouse_rangefinder.sdf`) live in the `ardupilot_gazebo` models and
worlds folders. The fault matrix needs neither: it runs on ArduPilot's
built-in physics.
