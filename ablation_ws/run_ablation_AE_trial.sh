#!/bin/bash
# --------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Re-runs ONE trial of the head-on ablation A to E of August 2026 (Table 4.2 / Table 7.2 of the
#   thesis) and, unlike the August recording, saves proof of what was running: the Nav2 parameter
#   file that was launched, the live parameters of the controller, the Nav2 log and the list of
#   running processes.
#
#   It runs in the separate workspace ablation_ws, which holds the August state:
#     - Nav2 parameter files, critic package, world, map, behaviour tree, yolo_detector and
#       lidar_person_fusion of commit eb3f9ac (15 Aug 2026)
#     - person cloud node, predictor and walker script as saved in the August bags
#       (cloud node with ELLIPSE_B = 1.00, which was never committed)
#   Protocol, read from the August bags (analysis/headon_ablation_AE_protocol.txt):
#     robot drives to (-1.9, 0), then gets the goal (8.5, 0); the walker goes from (8, 0) to
#     (-3, 0) at 1.2 m/s and starts 2.4 s after the robot (August: by hand, 2.4 +/- 1.0 s).
#   Differences to August (cannot be avoided, written into run_notes.txt):
#     every trial starts from a fresh Gazebo (no window); the walker is started by the script.
#
#   Run inside the container (the host script run_ablation_AE.sh does this):
#     ./run_ablation_AE_trial.sh <A|B|C|D|E> <bag_name>
#   Reads:  config/social_nav2_ablation_<condition>.yaml
#   Writes: bags/<bag_name>/ (bag, launched file, live parameters, logs, notes), logs/<bag_name>/
# --------------------------------------------------------------------------------------------------
set -e
[ $# -eq 2 ] || { echo "usage: $0 <A|B|C|D|E> <bag_name>" >&2; exit 1; }
COND=$1; NAME=$2

source /opt/ros/jazzy/setup.bash
source /root/thesis_social_navigation_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

WS=/root/thesis_social_navigation_ws
PKG="$WS/src/social_perception/social_perception"
case "$COND" in
  A) CFG="$WS/config/social_nav2_ablation_A_nolayer.yaml";                     CLOUD=false; EXP_W=20.0; EXP_SC=false ;;
  B) CFG="$WS/config/social_nav2_ablation_B_critweight20.yaml";                CLOUD=true;  EXP_W=20.0; EXP_SC=false ;;
  C) CFG="$WS/config/social_nav2_ablation_C_critweight10.yaml";                CLOUD=true;  EXP_W=10.0; EXP_SC=false ;;
  D) CFG="$WS/config/social_nav2_ablation_D_critweight10_socialcritic_on.yaml"; CLOUD=true;  EXP_W=10.0; EXP_SC=true ;;
  E) CFG="$WS/config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml"; CLOUD=true;  EXP_W=20.0; EXP_SC=true ;;
  *) echo "ERROR: condition must be A, B, C, D or E" >&2; exit 1 ;;
esac
WORLD=empty_human
MAP="$WS/maps/map_name.yaml"
BAG="$WS/bags/$NAME"
LOG_DIR="$WS/logs/$NAME"

# Start state of the August bags: odometry origin at about (-3.0, 1.0, 97 deg) in the map.
SPAWN_X=-3.0; SPAWN_Y=1.0; SPAWN_YAW=1.70
START_X=-1.9; START_Y=0.0          # August: robot start (-1.88 +/- 0.03, 0.03 +/- 0.03)
GOAL_X=8.5;  GOAL_Y=0.0            # August: robot end (8.51 +/- 0.08, -0.02 +/- 0.03)
PERSON_X0=8.0; PERSON_Y=0.0        # fixed inside the August walker script: (8, 0) -> (-3, 0), 1.2 m/s
MOVER_DELAY="${MOVER_DELAY:-2.4}"  # wall seconds after the goal is accepted (August mean)
GOAL_TIMEOUT="${GOAL_TIMEOUT:-180}"

[ -e "$BAG" ] && { echo "ERROR: $BAG already exists" >&2; exit 1; }
[ -f "$CFG" ] || { echo "ERROR: $CFG missing" >&2; exit 1; }
if pgrep -f "^gz sim" > /dev/null; then
  echo "ERROR: a Gazebo instance is already running - every trial needs a fresh launch" >&2; exit 1
fi
mkdir -p "$LOG_DIR"
cd "$WS"

PIDS=()
BAG_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo "[ablation] Tearing down..."
  if [ -n "$BAG_PID" ]; then
    kill -TERM -- "-$BAG_PID" 2>/dev/null || true   # rosbag2 closes the file on SIGTERM
    sleep 3
  fi
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 5
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  pkill -KILL -f "^gz sim" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start() {   # start <log name> <command...>   (own session, so teardown can signal the whole group)
  local log=$1; shift
  setsid "$@" > "$LOG_DIR/$log.log" 2>&1 &
  PIDS+=($!)
}
wait_active() {   # wait_active <lifecycle node> <seconds>
  local deadline=$((SECONDS + $2))
  while [ "$SECONDS" -lt "$deadline" ]; do
    state=$(timeout -k 2 10 ros2 lifecycle get "$1" 2>/dev/null | awk '{print $1}')
    [ "$state" = "active" ] && { echo "[ablation] $1 is active."; return 0; }
    sleep 1
  done
  echo "ERROR: $1 not active after $2 s" >&2
  return 1
}

# ----------------------------------------------------------------
# 1. Gazebo + TurtleBot4 + Nav2 + localization, with the condition's parameter file
# ----------------------------------------------------------------
[ -x /usr/local/bin/setup_gazebo_worlds.sh ] && /usr/local/bin/setup_gazebo_worlds.sh
timeout 10 ros2 daemon stop > /dev/null 2>&1 || true
pkill -KILL -f "ros2cli.daemon" 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null || true

# launch files with the `headless` argument (same files as in commit eb3f9ac, launch_overrides/)
SHARE_LAUNCH=/opt/ros/jazzy/share/turtlebot4_gz_bringup/launch
for f in sim.launch.py turtlebot4_gz.launch.py; do
  cmp -s "$WS/launch/$f" "$SHARE_LAUNCH/$f" || cp "$WS/launch/$f" "$SHARE_LAUNCH/$f"
done

echo "[ablation] Condition $COND: launching Gazebo + Nav2 with $(basename "$CFG") ..."
start nav2 ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  world:="$WORLD" slam:=false nav2:=true localization:=true rviz:=false \
  map:="$MAP" params_file:="$CFG" headless:=true \
  x:="$SPAWN_X" y:="$SPAWN_Y" yaw:="$SPAWN_YAW"

wait_active /map_server 120
wait_active /amcl 60
wait_active /controller_server 120

# ----------------------------------------------------------------
# 2. Initial pose, undock
# ----------------------------------------------------------------
sleep 5
INITIALPOSE_MSG=$(python3 -c "
import math
yaw = $SPAWN_YAW
print(f'{{header: {{frame_id: \"map\"}}, pose: {{covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942], pose: {{position: {{x: $SPAWN_X, y: $SPAWN_Y, z: 0.0}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}}}}}}}')")
LOCALIZED=false
for attempt in 1 2 3 4; do
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$INITIALPOSE_MSG" > /dev/null
  sleep 2
  for i in 1 2 3; do
    if timeout 4 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation"; then LOCALIZED=true; break; fi
    sleep 1
  done
  [ "$LOCALIZED" = true ] && break
  echo "[ablation] No map->odom after initial pose attempt $attempt, publishing again."
done
[ "$LOCALIZED" = true ] || { echo "ERROR: AMCL did not localize (no map->odom)" >&2; exit 1; }
echo "[ablation] AMCL localized."

UNDOCKED=false
for attempt in 1 2; do
  timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" > "$LOG_DIR/undock.log" 2>&1 || true
  grep -q "SUCCEEDED" "$LOG_DIR/undock.log" && { UNDOCKED=true; break; }
  echo "[ablation] Undock attempt $attempt did not succeed."
done
[ "$UNDOCKED" = true ] || { echo "ERROR: undock failed - see $LOG_DIR/undock.log" >&2; exit 1; }
sleep 3

# ----------------------------------------------------------------
# 3. Walker at its start point, then the August perception (perception_core.launch.py of eb3f9ac:
#    yolo_detector, human_kf_predictor, set_pose bridge and, except in A, the person cloud node),
#    and lidar_person_fusion, which gives the SocialCritic its person stream.
# ----------------------------------------------------------------
gz service -s "/world/$WORLD/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"person_1\", position: {x: $PERSON_X0, y: $PERSON_Y, z: 0.0}, orientation: {z: -0.70710678, w: 0.70710678}" \
  > /dev/null || { echo "ERROR: could not place person_1 at the start point" >&2; exit 1; }

start perception ros2 launch "$WS/launch/perception_core.launch.py" ws_root:="$WS" world:="$WORLD" \
  enable_predicted_cloud:="$CLOUD"
start lidar_fusion python3 -u "$PKG/lidar_person_fusion.py" --ros-args -p use_sim_time:=true

echo "[ablation] Waiting for the perception pipeline..."
READY=false
for i in $(seq 1 120); do
  if [ "$(ros2 topic info /person_positions_map 2>/dev/null | awk '/Publisher count/ {print $3}')" = "1" ] \
     && [ "$(ros2 topic info /predicted_person_positions 2>/dev/null | awk '/Publisher count/ {print $3}')" = "1" ] \
     && [ "$(ros2 topic info /fused_person_positions 2>/dev/null | awk '/Publisher count/ {print $3}')" = "1" ] \
     && ros2 service list 2>/dev/null | grep -q "/world/$WORLD/set_pose"; then
    READY=true; break
  fi
  sleep 1
done
[ "$READY" = true ] || { echo "ERROR: perception pipeline not ready - see $LOG_DIR/perception.log" >&2; exit 1; }
N_CLOUD_PUB=$(ros2 topic info /predicted_person_cloud 2>/dev/null | awk '/Publisher count/ {print $3}')
if [ "$CLOUD" = true ]; then
  [ "$N_CLOUD_PUB" = "1" ] || { echo "ERROR: cloud node should run in condition $COND but /predicted_person_cloud has ${N_CLOUD_PUB:-0} publishers" >&2; exit 1; }
else
  [ "${N_CLOUD_PUB:-0}" = "0" ] || { echo "ERROR: condition A but /predicted_person_cloud has $N_CLOUD_PUB publishers" >&2; exit 1; }
fi
sleep 8   # YOLO model warm-up
echo "[ablation] Perception ready (person cloud node: $CLOUD)."

# ----------------------------------------------------------------
# 4. Drive to the start pose of the August trials
# ----------------------------------------------------------------
echo "[ablation] Driving to the start pose ($START_X, $START_Y)..."
python3 "$WS/headon_send_goal.py" "$START_X" "$START_Y" 120 "$LOG_DIR/predrive_accepted" > "$LOG_DIR/predrive_result.txt" 2>&1 || true
grep -q "status: SUCCEEDED" "$LOG_DIR/predrive_result.txt" || { echo "ERROR: start pose not reached: $(tail -1 "$LOG_DIR/predrive_result.txt")" >&2; exit 1; }
sleep 4

# ----------------------------------------------------------------
# 5. Proof of what is running, then the bag
# ----------------------------------------------------------------
mkdir -p "$BAG"
cp "$CFG" "$BAG/config_used.yaml"                       # the file that was passed to the launch
echo "$CFG" > "$BAG/config_path.txt"
tr '\0' ' ' < "/proc/${PIDS[0]}/cmdline" > "$BAG/nav2_launch_cmdline.txt" 2>/dev/null || true
cp "$PKG/predicted_person_cloud_node.py" "$BAG/cloud_node_used.py"
cp "$PKG/human_kf_predictor.py" "$BAG/kf_used.py"
cp "$PKG/move_person_oneway.py" "$BAG/mover_used.py"
cp "$PKG/yolo_detector.py" "$BAG/yolo_used.py"
cp "$PKG/lidar_person_fusion.py" "$BAG/lidar_fusion_used.py"
cp "$WS/launch/perception_core.launch.py" "$BAG/launch_used.py"
cp "$WS/src/social_critic/src/social_critic.cpp" "$BAG/social_critic_used.cpp"
cp "$WS/src/social_critic/include/social_critic/social_critic.hpp" "$BAG/social_critic_used.hpp"
cp "$WS/behavior_trees/navigate_to_pose_fast_replan.xml" "$BAG/bt_used.xml"
cp "/opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds/$WORLD.sdf" "$BAG/world_used.sdf"
cp "$MAP" "$BAG/map_used.yaml"
cp "$0" "$BAG/trial_script_used.sh"
cp "$WS/headon_send_goal.py" "$BAG/send_goal_used.py"
ps -eo args | grep -E "[y]olo_detector.py|[h]uman_kf_predictor.py|[p]redicted_person_cloud_node.py|[l]idar_person_fusion.py|[p]arameter_bridge" > "$BAG/processes_running.txt" || true
for n in /controller_server /planner_server /bt_navigator /local_costmap/local_costmap /global_costmap/global_costmap; do
  f="$BAG/params_live$(echo "$n" | tr '/' '_').yaml"
  for try in 1 2 3; do
    timeout 20 ros2 param dump "$n" > "$f" 2>/dev/null || true
    [ -s "$f" ] && grep -q "ros__parameters" "$f" && break
    sleep 2
  done
  [ -s "$f" ] || echo "WARN: param dump empty for $n" >&2
done
# The three settings that define the condition, read from the running controller.
LIVE_W=$(timeout 15 ros2 param get /controller_server FollowPath.ObstaclesCritic.critical_weight 2>/dev/null | awk '{print $NF}')
LIVE_SC=$(timeout 15 ros2 param get /controller_server FollowPath.SocialCritic.enabled 2>/dev/null | awk '{print tolower($NF)}')
LIVE_CRITICS=$(timeout 15 ros2 param get /controller_server FollowPath.critics 2>/dev/null | tr '\n' ' ')
LIVE_GLAYERS=$(timeout 15 ros2 param get /global_costmap/global_costmap plugins 2>/dev/null | tr '\n' ' ')
LIVE_LLAYERS=$(timeout 15 ros2 param get /local_costmap/local_costmap plugins 2>/dev/null | tr '\n' ' ')
find "$WS/install" -name "libsocial_critic.so" -exec md5sum {} + > "$BAG/critic_so_md5.txt" || true
md5sum "$BAG"/*_used.* > "$BAG/provenance_md5.txt"
CHECK=OK
[ "$LIVE_W" = "$EXP_W" ] || CHECK="MISMATCH (critical_weight live '$LIVE_W', expected $EXP_W)"
[ "$LIVE_SC" = "$EXP_SC" ] || CHECK="$CHECK MISMATCH (SocialCritic.enabled live '$LIVE_SC', expected $EXP_SC)"
cat > "$BAG/run_notes.txt" <<NOTES
Condition: $COND   bag: $NAME
Scenario: head-on ablation A-E, world $WORLD, re-run of the August 2026 trials with the August state restored.
Nav2 params file launched: $CFG (md5 $(md5sum < "$CFG" | cut -d' ' -f1))
Person cloud node started: $CLOUD   (publishers on /predicted_person_cloud: ${N_CLOUD_PUB:-0})
Live controller, ObstaclesCritic.critical_weight: $LIVE_W   (expected $EXP_W)
Live controller, SocialCritic.enabled: $LIVE_SC   (expected $EXP_SC)
Live controller, critics: $LIVE_CRITICS
Live global costmap layers: $LIVE_GLAYERS
Live local costmap layers: $LIVE_LLAYERS
Condition check: $CHECK
Robot spawn: ($SPAWN_X, $SPAWN_Y, yaw $SPAWN_YAW) on the dock, undock, Nav2 goal to the start pose ($START_X, $START_Y), then goal ($GOAL_X, $GOAL_Y)
Walker: ($PERSON_X0, $PERSON_Y) -> (-3, 0) at 1.2 m/s (fixed in the August walker script), started $MOVER_DELAY s (wall) after the goal was accepted
Differences to August: fresh Gazebo without window for every trial; walker started by the script, not by hand; lidar_person_fusion runs in all conditions.
NOTES
echo "[ablation] Condition check: $CHECK  (weight $LIVE_W, SocialCritic $LIVE_SC, cloud node $CLOUD)"
case "$CHECK" in OK) ;; *) echo "ERROR: the running parameters are not those of condition $COND" >&2; exit 1 ;; esac

echo "[ablation] Recording to $BAG/data ..."
setsid ros2 bag record \
  --qos-profile-overrides-path "$WS/analysis/tf_qos_override.yaml" \
  --topics /clock /person_ground_truth /odom /amcl_pose /tf /tf_static /plan \
           /predicted_person_positions /person_positions_map /fused_person_positions /predicted_person_cloud \
           /cmd_vel /cmd_vel_smoothed /cmd_vel_nav /collision_monitor_state \
           /optimal_trajectory /rosout \
  -o "$BAG/data" > "$LOG_DIR/bag.log" 2>&1 &
BAG_PID=$!
sleep 3

# ----------------------------------------------------------------
# 6. Goal, then the walker
# ----------------------------------------------------------------
echo "[ablation] Sending the goal ($GOAL_X, $GOAL_Y)..."
FLAG="$LOG_DIR/goal_accepted"
rm -f "$FLAG"
python3 "$WS/headon_send_goal.py" "$GOAL_X" "$GOAL_Y" "$GOAL_TIMEOUT" "$FLAG" > "$BAG/goal_result.txt" 2>&1 &
GOAL_PID=$!
for i in $(seq 1 300); do
  [ -f "$FLAG" ] && break
  kill -0 "$GOAL_PID" 2>/dev/null || break
  sleep 0.1
done
if [ -f "$FLAG" ]; then
  sleep "$MOVER_DELAY"
  echo "[ablation] Goal accepted - walker started after $MOVER_DELAY s."
  start mover python3 "$PKG/move_person_oneway.py" --ros-args -p use_sim_time:=true
fi
set +e
wait "$GOAL_PID"
set -e
cat "$BAG/goal_result.txt"
sleep 3

# ----------------------------------------------------------------
# 7. Result and logs
# ----------------------------------------------------------------
DIED=0
for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null || DIED=$((DIED + 1)); done
cp "$LOG_DIR/nav2.log" "$BAG/nav2_log.txt" 2>/dev/null || true          # includes the controller log
cp "$LOG_DIR/perception.log" "$BAG/perception_log.txt" 2>/dev/null || true
grep -h -E "Critic loaded|SocialCritic" "$LOG_DIR/nav2.log" | head -40 > "$BAG/critics_loaded.txt" || true
echo "Processes that died before the end: $DIED" >> "$BAG/run_notes.txt"
echo "Goal result: $(tail -1 "$BAG/goal_result.txt")" >> "$BAG/run_notes.txt"
if grep -q "status: SUCCEEDED" "$BAG/goal_result.txt"; then
  echo "[ablation] RESULT: goal succeeded (processes died: $DIED)."
else
  echo "[ablation] RESULT: goal did NOT succeed: $(tail -1 "$BAG/goal_result.txt") (processes died: $DIED)."
fi
