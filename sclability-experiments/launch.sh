#!/usr/bin/env bash
# Automated scalability experiments for TurtleBot flocking
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/ros_ws}"
CONFIG_SET="${1:-no-threads}"
SCALABILITY_DIR="$ROS_WS/src/turtlebot_flocking/sclability-experiments/${CONFIG_SET}"
if [[ ! -d "$SCALABILITY_DIR" ]]; then
  echo "[ERROR] Config directory $SCALABILITY_DIR not found."
  exit 1
fi
CONFIG_YAML="$ROS_WS/src/turtlebot_flocking/config/config.yaml"
COUNTS=(10 20 40 80 160 320 640 1280)
REPETITIONS=10
NODES_PER_DOMAIN=50
ARGOS_BIN="${ARGOS_BIN:-argos3}"
LOG_ROOT="$ROS_WS/scalability_runs/${CONFIG_SET}_$(date +%Y%m%d_%H%M%S)"
TMP_DIR=$(mktemp -d /tmp/turtlebot_scalability.XXXXXX)

mkdir -p "$LOG_ROOT"

cleanup() {
  trap - INT TERM EXIT
  pkill -f "ros2 launch" 2>/dev/null || true
  pkill -f "turtlebot_flocking" 2>/dev/null || true
  pkill -f "$ARGOS_BIN" 2>/dev/null || true
  rm -rf "$TMP_DIR"
}
trap cleanup INT TERM EXIT

reset_ros_environment() {
  pkill -f "ros2 launch" 2>/dev/null || true
  pkill -f "turtlebot_flocking" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

source "$ROS_WS/install/setup.bash"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:"$ROS_WS"/install/argos3_ros2_bridge/lib
export ARGOS_PLUGIN_PATH="$ROS_WS"/install/argos3_ros2_bridge/lib
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

generate_launch_file() {
  local count=$1
  local launch_file="$TMP_DIR/controllers_${count}.launch.py"
  cat > "$launch_file" <<PY
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    with open("${CONFIG_YAML}", 'r', encoding='utf-8') as cfg:
        params = yaml.safe_load(cfg)["flocking"]["ros__parameters"]
    ld = LaunchDescription()
    for idx in range(${count}):
        namespace = f"bot{idx}"
        domain = idx // ${NODES_PER_DOMAIN}
        ld.add_action(Node(
            package="argos3_ros2_bridge",
            executable="turtlebot_flocking",
            name="turtlebot_flocking",
            namespace=namespace,
            output="log",
            parameters=[params, {"robot_namespace": f"/{namespace}"}],
            additional_env={"ROS_DOMAIN_ID": str(domain)}
        ))
    return ld
PY
  echo "$launch_file"
}

run_trial() {
  local count=$1
  local rep=$2
  local launch_file=$3
  local config_file="$SCALABILITY_DIR/flocking${count}.argos"
  local run_dir="$LOG_ROOT/${count}_robots/run_${rep}"
  mkdir -p "$run_dir"
  echo "[INFO] Robots: $count | Repetition: $rep"

  ros2 launch "$launch_file" > "$run_dir/ros2.log" 2>&1 &
  local ros2_pid=$!
  sleep 5

  (cd "$SCALABILITY_DIR" && $ARGOS_BIN -c "$config_file" -z) > "$run_dir/argos.log" 2>&1

  if kill -0 $ros2_pid 2>/dev/null; then
    kill -INT $ros2_pid 2>/dev/null || true
    wait $ros2_pid 2>/dev/null || true
  fi

  local metrics_file="$SCALABILITY_DIR/sync_metrics_turtlebot_${count}.csv"
  if [[ -f "$metrics_file" ]]; then
    cp "$metrics_file" "$run_dir/sync_metrics.csv"
  else
    echo "[WARN] Metrics file missing." | tee -a "$run_dir/notes.txt"
  fi
}

cd "$SCALABILITY_DIR"

for count in "${COUNTS[@]}"; do
  config="$SCALABILITY_DIR/flocking${count}.argos"
  if [[ ! -f "$config" ]]; then
    echo "[WARN] Missing config for ${count} robots, skipping."
    continue
  fi
  launch_file=$(generate_launch_file "$count")
  for rep in $(seq 1 "$REPETITIONS"); do
    run_trial "$count" "$rep" "$launch_file"
  done
  rm -f "$launch_file"
  reset_ros_environment
  echo "[INFO] Completed ${count}-robot batch."
done

echo "[INFO] All experiments complete. Logs stored in $LOG_ROOT"
