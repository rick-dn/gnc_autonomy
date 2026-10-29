#!/bin/bash
# start_estimator.sh - Opens terminals for recording custom-estimator flights
# Usage: bash scripts/start_estimator.sh
#
# Same stack as start_rangfinder.sh, but SITL runs from runs/estimator with its
# own eeprom.bin and logs, and loads est_logging.parm (400 Hz IMU logging).
# Nothing here carries over to the P1 setup in runs/sitl.

PROJECT_DIR="/data/projects/udl_aakash"
ARDUPILOT_DIR="/data/projects/udl_aakash/tools/ardupilot/Tools/autotest"
GNC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$GNC_DIR/runs/estimator"
PARAM_FILE="$GNC_DIR/ga_sim/params/base.parm"
EST_PARAM_FILE="$GNC_DIR/ga_sim/params/est_logging.parm"
MAVROS_PARAMS="$GNC_DIR/ga_sim/params/mavros_rangefinder.yaml"

mkdir -p "$RUN_DIR"

# Tab 1: Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "cd $PROJECT_DIR && gz sim -v4 -r iris_warehouse_rangefinder.sdf; bash"

# Small delay so the world is loaded before SITL connects to the plugin
sleep 5

# Tab 2: ArduPilot
gnome-terminal --tab --title="ArduPilot" -- bash -c "cd $RUN_DIR && $ARDUPILOT_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --add-param-file=$PARAM_FILE --add-param-file=$EST_PARAM_FILE --map --console -L Basingstoke; bash"

# Small delay before starting dependent services
sleep 3

# Tab 3: MAVROS
gnome-terminal --tab --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args -p system_id:=200 -p fcu_url:=udp://:14550@localhost:14555 --params-file $MAVROS_PARAMS; bash"
