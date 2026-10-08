#!/bin/bash
# hw_replay.sh <extracted bag name> <nav2 config> <tag> - replay a robot bag through the real SocialCritic (8 Oct 2026).
# Run inside the simulation container. Open loop: the robot's motion is the recorded one; only the critic's
# decisions (lane start, side, side fixed, jumps, carry-over) are produced, by the same C++ plugin the simulation uses.
#   bag:    bags/hw_replay_src/<name>/data  (from analysis/hw_replay_extract.py)
#   output: analysis/hw_replay/out/<tag>/<name>/ controller.log, decisions.txt (replay vs the robot's own /rosout)
# Own DDS domain (42), nothing else is touched.
# (no set -u: the ROS setup scripts use unset variables)
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
NAME=$1; CFG=$2; TAG=$3
# Leftovers of an earlier replay would answer on the same names: stop them first (by PID, never by pattern-kill).
for p in $(ps -eo pid,args | grep -E "[c]ontroller_server --ros-args --params-file $WS/analysis/hw_replay|[p]ath_feeder.py" | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
BAG="$WS/bags/hw_replay_src/$NAME/data"
OUT="$WS/analysis/hw_replay/out/$TAG/$NAME"; mkdir -p "$OUT"
P="$OUT/replay_params.yaml"
python3 analysis/hw_replay/make_replay_params.py "$CFG" "$P" > /dev/null || exit 1
cp "$CFG" "$OUT/config_used.yaml"
md5sum install/social_critic/lib/libsocial_critic.so > "$OUT/critic_so_md5.txt"
/opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args --params-file "$P" -p use_sim_time:=true \
  -r cmd_vel:=/replay_cmd_vel -r cmd_vel_nav:=/replay_cmd_vel > "$OUT/controller.log" 2>&1 &
C=$!
python3 analysis/hw_replay/path_feeder.py > "$OUT/feeder.log" 2>&1 &
F=$!
# Bag first (TF must flow before the costmap can activate), then the lifecycle manager.
ros2 bag play "$BAG" --clock 50 --remap /turtlebot4/tf:=/tf /turtlebot4/tf_static:=/tf_static \
  /turtlebot4/odom:=/odom /turtlebot4/scan:=/scan /turtlebot4/plan:=/bag_plan /rosout:=/bag_rosout \
  /turtlebot4/goal_pose:=/bag_goal_pose > "$OUT/play.log" 2>&1 &
B=$!
# Bring the controller up with retries (the lifecycle manager hung or timed out while TF was not yet there).
L=""
for i in $(seq 1 40); do
  st=$(ros2 lifecycle get /controller_server 2>/dev/null | awk '{print $1}')
  case "$st" in
    active) echo "controller active after $i checks" > "$OUT/lifecycle.log"; break ;;
    unconfigured) ros2 lifecycle set /controller_server configure > /dev/null 2>&1 ;;
    inactive) ros2 lifecycle set /controller_server activate > /dev/null 2>&1 ;;
  esac
  sleep 0.3
done
[ "$st" = active ] || echo "controller NOT active (state: $st)" >> "$OUT/lifecycle.log"
wait $B
sleep 2
kill -INT $F $C 2>/dev/null; sleep 3; kill -9 $F $C 2>/dev/null
python3 analysis/hw_replay/compare_decisions.py "$BAG" "$OUT/controller.log" > "$OUT/decisions.txt"
cat "$OUT/decisions.txt"
