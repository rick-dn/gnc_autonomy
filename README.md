# GNC Autonomy

## Project 1: GPS-denied baseline

Fly the quad in ArduPilot SITL with GPS off, using Gazebo's ground-truth pose
as a fake "SLAM" input sent to ArduPilot as external nav. The result is a
working harness where later projects only swap the pose source.

## Layout

```
gnc_autonomy/
├── ga_ws/src/
│   ├── ga_vehicle/         # MAVROS adapter and mission scripts
│   ├── ga_extnav_bridge/   # pose source -> external nav for ArduPilot
│   └── ga_bringup/         # launch files and ArduPilot param files
├── tools/eval/             # evaluation scripts and plots
└── docs/                   # documentation, plots, GIF
```
