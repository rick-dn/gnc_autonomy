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

### Next

- More EKF3 faults: GPS glitch, IMU bias, compass errors
- External nav from ground-truth pose, with noise, delay and source switching
- C2 link-loss failsafes

## Layout

```
gnc_autonomy/
├── scripts/                # launchers (start_rangfinder.sh)
├── ga_sim/params/          # ArduPilot and MAVROS param files
├── ga_ws/src/
│   ├── ga_vehicle/         # MAVROS adapter, source_switcher node
│   ├── ga_extnav_bridge/   # pose source -> external nav (planned)
│   └── ga_bringup/         # launch files (planned)
├── tools/eval/             # log analysis and plots
└── docs/                   # write-ups and figures
```

The Gazebo model (`iris_with_rangefinder`) and world
(`iris_warehouse_rangefinder.sdf`) live in the `ardupilot_gazebo` models and
worlds folders.
