#!/bin/bash
# start_rangfinder.sh - Opens terminals for the rangefinder SITL stack
# (iris_with_rangefinder in iris_warehouse_rangefinder.sdf)
# Usage: bash scripts/start_rangfinder.sh
#
# ArduPilot runs from gnc_autonomy/runs/sitl so its eeprom.bin and logs stay
# separate from udl_aakash's start_all.sh, and loads base.parm (sensors) and
# p1_sources.parm (EKF3 source sets) on top of the stock gazebo-iris params.
# The ArduPilot, Gazebo and MAVROS installs are udl_aakash's, used as-is.

PROJECT_DIR="/data/projects/udl_aakash"
ARDUPILOT_DIR="/data/projects/udl_aakash/tools/ardupilot/Tools/autotest"
GNC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$GNC_DIR/runs/sitl"
PARAM_FILE="$GNC_DIR/ga_sim/params/base.parm"
P1_PARAM_FILE="$GNC_DIR/ga_sim/params/p1_sources.parm"
MAVROS_PARAMS="$GNC_DIR/ga_sim/params/mavros_rangefinder.yaml"

mkdir -p "$RUN_DIR"

# Tab 1: Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "cd $PROJECT_DIR && gz sim -v4 -r iris_warehouse_rangefinder.sdf; bash"

# Small delay so the world is loaded before SITL connects to the plugin
sleep 5

# Tab 2: ArduPilot
gnome-terminal --tab --title="ArduPilot" -- bash -c "cd $RUN_DIR && $ARDUPILOT_DIR/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild --add-param-file=$PARAM_FILE --add-param-file=$P1_PARAM_FILE --map --console -L Basingstoke; bash"

# Small delay before starting dependent services
sleep 3

# Tab 3: MAVROS
gnome-terminal --tab --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args -p system_id:=200 -p fcu_url:=udp://:14550@localhost:14555 --params-file $MAVROS_PARAMS; bash"
