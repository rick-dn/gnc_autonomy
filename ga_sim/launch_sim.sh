#!/bin/bash

# Launch Gazebo + ArduPilot SITL for gnc_autonomy.
# Usage: bash ga_sim/launch_sim.sh [--headless] [--world FILE] [--wipe] [-- sim_vehicle args...]
#
#   --headless   Gazebo server only, no window (rendering still runs via EGL
#                so the rangefinder works)
#   --world F    world file on GZ_SIM_RESOURCE_PATH
#                (default iris_warehouse_rangefinder.sdf)
#   --wipe       wipe eeprom (-w) so params reload from the param files
#   --           everything after is passed to sim_vehicle.py as-is,
#                e.g. -- --no-mavproxy
#
# SITL runs from runs/sitl/ in the project root, so eeprom.bin, logs and
# scripts/ land here and not in the ArduPilot tree. Gazebo's log goes to
# the same folder. Ctrl-C stops both.

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="/data/projects/udl_aakash/tools/ardupilot"
GZ_DIR="/data/projects/udl_aakash/ros_ws/src/ardupilot_gazebo"
GZ_PLUGIN_DIR="/data/projects/udl_aakash/ros_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo"
RUN_DIR="$ROOT/runs/sitl"
PARAMS="$ROOT/ga_sim/params/base.parm"

WORLD="iris_warehouse_rangefinder.sdf"
HEADLESS=0
WIPE=0
SIM_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS=1; shift ;;
        --world)    WORLD="$2"; shift 2 ;;
        --wipe)     WIPE=1; shift ;;
        --)         shift; SIM_ARGS=("$@"); break ;;
        *) echo "❌ Unknown option: $1"; exit 1 ;;
    esac
done

# Only set for this process - fall back to the ardupilot_gazebo paths if the
# shell didn't already provide them.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-$GZ_DIR/models:$GZ_DIR/worlds}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-$GZ_PLUGIN_DIR}"

mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

GZ_ARGS=(-r -v3)
if [[ "$HEADLESS" == "1" ]]; then
    GZ_ARGS=(-s --headless-rendering "${GZ_ARGS[@]}")
fi

echo "🌍 Starting Gazebo: $WORLD (log: $RUN_DIR/gz.log)"
gz sim "${GZ_ARGS[@]}" "$WORLD" > gz.log 2>&1 &
GZ_PID=$!
trap 'echo "🛑 Stopping Gazebo"; kill $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null' EXIT

# Let Gazebo load the world before SITL connects to the plugin.
sleep 5
if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "❌ Gazebo exited - see $RUN_DIR/gz.log"
    exit 1
fi

SITL_ARGS=(-v ArduCopter -f gazebo-iris --model JSON --no-rebuild
           --add-param-file="$PARAMS")
if [[ "$WIPE" == "1" ]]; then
    SITL_ARGS+=(-w)
fi

echo "🚁 Starting SITL in $RUN_DIR"
"$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" "${SITL_ARGS[@]}" "${SIM_ARGS[@]}"
