#!/usr/bin/env bash
# Automated scalability experiments for TurtleBot flocking
set -euo pipefail

ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
CONFIG_SET="${1:-no-threads}"
SCALABILITY_DIR="$ROS_WS/src/turtlebot-flocking/scalability-experiments/${CONFIG_SET}"
if [[ ! -d "$SCALABILITY_DIR" ]]; then
  echo "[ERROR] Config directory $SCALABILITY_DIR not found."
  exit 1
fi
CONFIG_YAML="$ROS_WS/src/turtlebot-flocking/config/config.yaml"
COUNTS=(40 80 160 320 640 1280)
REPETITIONS=10
NODES_PER_DOMAIN=50
ARGOS_BIN="${ARGOS_BIN:-argos3}"
LOG_BASE="${LOG_BASE:-$ROS_WS/scalability_runs}"
LOG_ROOT="$LOG_BASE/${CONFIG_SET}_$(date +%Y%m%d_%H%M%S)"
TMP_DIR=$(mktemp -d /tmp/turtlebot_scalability.XXXXXX)
TIMEOUT_SECONDS=20
SHUTDOWN_GRACE_SECONDS=5
SAMPLE_INTERVAL=1
NUM_CORES=$(nproc --all)
RESULTS_FILE="$LOG_ROOT/summary.csv"
ROS_STARTUP_DELAY="${ROS_STARTUP_DELAY:-5}"

mkdir -p "$LOG_ROOT"
echo "Robots,Repetition,WallTime(s),CPU(%),MaxMem(kB),ArgosCPU(%),Ros2CPU(%),ArgosMem(kB),Ros2Mem(kB)" > "$RESULTS_FILE"

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
  pkill -f "argos3" 2>/dev/null || true
  sleep 1
  pkill -9 -f "turtlebot_flocking" 2>/dev/null || true
  pkill -9 -f "argos3" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
}

set +u
source "$ROS_WS/install/setup.bash"
set -u
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
  local count_dir="$LOG_ROOT/${count}_robots"
  mkdir -p "$count_dir"
  local run_dir="$count_dir/run_${rep}"
  mkdir -p "$run_dir"

  local ros2_log="$run_dir/ros2.log"
  local argos_log="$run_dir/argos.log"
  local debug_log="$run_dir/debug.log"
  local ros2_time_file="$run_dir/ros2_time.tmp"
  local argos_time_file="$run_dir/argos_time.tmp"
  local population_csv="$count_dir/results.csv"
  if [[ ! -f "$population_csv" ]]; then
    echo "Robots,Repetition,WallTime(s),CPU(%),MaxMem(kB),ArgosCPU(%),Ros2CPU(%),ArgosMem(kB),Ros2Mem(kB)" > "$population_csv"
  fi

  echo "[INFO] Robots: $count | Repetition: $rep"
  local start_time=$(date +%s.%N)

  ( /usr/bin/time -f "%e %P %M" -o "$ros2_time_file" ros2 launch "$launch_file" ) > "$ros2_log" 2>&1 &
  local ros2_pid=$!
  sleep "$ROS_STARTUP_DELAY"

  /usr/bin/time -f "%e %P %M" -o "$argos_time_file" $ARGOS_BIN -c "$config_file" -z > "$argos_log" 2>&1 &
  local argos_pid=$!

  local ros2_cpu_total=0
  local ros2_mem_peak=0
  local sample_count=0

  while true; do
    if ! kill -0 $argos_pid 2>/dev/null; then
      break
    fi
    local ros2_cpu_sum=0
    local ros2_mem_sum=0
    local active_nodes=0
    local node_list=$(ros2 node list 2>/dev/null | grep -c "/bot[0-9]" || true)
    local pids
    pids=$(pgrep -f "turtlebot_flocking" 2>/dev/null || true)
    for pid in $pids; do
      [[ -z "$pid" ]] && continue
      local metrics
      metrics=$(ps -p "$pid" -o %cpu= -o rss= 2>/dev/null) || continue
      local cpu=$(echo "$metrics" | awk '{print $1}')
      local mem=$(echo "$metrics" | awk '{print $2}')
      ros2_cpu_sum=$(echo "$ros2_cpu_sum + ${cpu:-0}" | bc -l)
      ros2_mem_sum=$(echo "$ros2_mem_sum + ${mem:-0}" | bc -l)
      active_nodes=$((active_nodes + 1))
    done

    local ros2_cpu_avg=0
    if (( $(echo "$ros2_cpu_sum > 0" | bc -l) )); then
      ros2_cpu_avg=$(echo "scale=2; $ros2_cpu_sum / ${NUM_CORES}" | bc -l)
    fi
    ros2_cpu_total=$(echo "$ros2_cpu_total + $ros2_cpu_avg" | bc -l)
    if (( $(echo "$ros2_mem_sum > $ros2_mem_peak" | bc -l) )); then
      ros2_mem_peak=$ros2_mem_sum
    fi
    sample_count=$((sample_count + 1))
    printf "Sample %03d: nodes=%d (ros2 list: %s) CPU=%s%% Mem=%s kB\n" \
      "$sample_count" "$active_nodes" "$node_list" "$ros2_cpu_avg" "$ros2_mem_sum" >> "$debug_log"
    sleep $SAMPLE_INTERVAL
  done

  local argos_status=0
  if ! wait $argos_pid 2>/dev/null; then
    argos_status=$?
  fi

  if kill -0 $ros2_pid 2>/dev/null; then
    kill -INT $ros2_pid 2>/dev/null || true
    local waited=0
    while kill -0 $ros2_pid 2>/dev/null; do
      if (( waited >= TIMEOUT_SECONDS )); then
        kill -TERM $ros2_pid 2>/dev/null || true
        sleep $SHUTDOWN_GRACE_SECONDS
        kill -KILL $ros2_pid 2>/dev/null || true
        break
      fi
      sleep 1
      waited=$((waited + 1))
    done
  fi
  local ros2_status=0
  if ! wait $ros2_pid 2>/dev/null; then
    ros2_status=$?
  fi

  # Force kill any remaining controllers/argos processes before continuing
  pkill -f "turtlebot_flocking" 2>/dev/null || true
  pkill -f "$ARGOS_BIN" 2>/dev/null || true
  sleep 1
  pkill -9 -f "turtlebot_flocking" 2>/dev/null || true
  pkill -9 -f "$ARGOS_BIN" 2>/dev/null || true
  while pgrep -f "turtlebot_flocking" >/dev/null; do sleep 1; done
  while pgrep -f "$ARGOS_BIN" >/dev/null; do sleep 1; done

  local run_failed=false
  local failure_reason=""
  if [[ $argos_status -ne 0 ]]; then
    run_failed=true
    failure_reason="argos exit $argos_status"
  fi
  if [[ $ros2_status -ne 0 ]]; then
    run_failed=true
    if [[ -n "$failure_reason" ]]; then
      failure_reason+="; "
    fi
    failure_reason+="ros2 exit $ros2_status"
  fi

  local argos_wall=0 argos_cpu=0 argos_mem=0
  if [[ -f "$argos_time_file" ]]; then
    read -r argos_wall argos_cpu argos_mem < "$argos_time_file"
    argos_cpu=${argos_cpu//%/}
    rm -f "$argos_time_file"
  fi

  local ros2_wall=0 ros2_cpu=0 ros2_mem=0
  if [[ -f "$ros2_time_file" && -s "$ros2_time_file" ]]; then
    read -r ros2_wall ros2_cpu ros2_mem < "$ros2_time_file"
    ros2_cpu=${ros2_cpu//%/}
    rm -f "$ros2_time_file"
  elif (( sample_count > 0 )); then
    ros2_cpu=$(printf "%.2f" $(echo "$ros2_cpu_total / $sample_count" | bc -l))
    ros2_mem=$ros2_mem_peak
  fi

  local wall_time=0
  if (( $(echo "$argos_wall > 0" | bc -l) )) && (( $(echo "$ros2_wall > 0" | bc -l) )); then
    if (( $(echo "$argos_wall > $ros2_wall" | bc -l) )); then
      wall_time=$argos_wall
    else
      wall_time=$ros2_wall
    fi
  elif (( $(echo "$argos_wall > 0" | bc -l) )); then
    wall_time=$argos_wall
  elif (( $(echo "$ros2_wall > 0" | bc -l) )); then
    wall_time=$ros2_wall
  else
    wall_time=$(echo "scale=2; $(date +%s.%N) - $start_time" | bc -l)
  fi

  local total_cpu=$(printf "%.2f" $(echo "${argos_cpu:-0} + ${ros2_cpu:-0}" | bc -l))
  local total_mem=$(printf "%.0f" $(echo "${argos_mem:-0} + ${ros2_mem:-0}" | bc -l))

  if [[ $run_failed == true ]]; then
    echo "[WARN] Run failed for ${count} robots repetition ${rep}: ${failure_reason}" | tee -a "$run_dir/notes.txt"
  fi

  local result_line="$count,$rep,$wall_time,$total_cpu,$total_mem,${argos_cpu:-0},${ros2_cpu:-0},${argos_mem:-0},${ros2_mem:-0}"
  echo "$result_line" >> "$RESULTS_FILE"
  echo "$result_line" >> "$population_csv"

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
