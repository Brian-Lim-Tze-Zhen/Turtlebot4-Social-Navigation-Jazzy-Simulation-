#!/bin/bash
# hw_perception_replay.sh <bag> <profile> <mode> <nav2 config> <tag> - replay a robot bag through the robot's
# perception chain AND the critic (8 Oct 2026). Inside the simulation container, own DDS domain (43).
#   bag:     bags/hw_replay_src_id/<bag>/data (hw_replay_extract.py with EXTRA_TOPICS)
#   profile: base | lane | noghost   (launch settings of that robot day: identity coasting, KF predictor)
#   mode:    orig | fix              (orig: verbatim copies of the robot nodes; fix: the *_fix.py copies with
#                                     still_split_m 0.4 and motion_release_max_turn_deg 5.0)
# Chain: bag (/person_positions_map, scan, odom, tf, map) -> ray node -> identity node -> KF -> critic
# (controller_server as in hw_replay.sh). The bag's own perception outputs are remapped to /bag_* and recorded
# next to the replayed ones for the comparison. Robot nodes in the robot repo are never touched.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
NAME=$1; PROF=$2; MODE=$3; CFG=$4; TAG=$5
D=analysis/hw_replay/perception
BAG="$WS/bags/hw_replay_src_id/$NAME/data"
OUT="$WS/analysis/hw_replay/out/$TAG/$NAME"; rm -rf "$OUT"; mkdir -p "$OUT"
for p in $(ps -eo pid,args | grep -E "[c]ontroller_server --ros-args --params-file $WS/analysis/hw_replay|[p]ath_feeder.py|$D/[a-z_]*\.py|[c]hain_logger.py" | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
case $PROF in
  base)    COAST=false; KF=human_kf_predictor_lidar ;;
  lane)    COAST=true;  KF=human_kf_predictor_lidar ;;
  noghost) COAST=true;  KF=human_kf_predictor_lidar_noghost ;;
  *) echo "profile?"; exit 1 ;;
esac
if [ "$MODE" = fix ]; then
  RAY="$D/camera_ray_person_node_fix.py -p motion_release_max_turn_deg:=5.0"
  IDN="$D/camera_ray_identity_node_fix.py -p still_split_m:=0.4"
else
  RAY="$D/camera_ray_person_node_orig.py"; IDN="$D/camera_ray_identity_node_orig.py"
fi
echo "profile $PROF mode $MODE kf $KF config $(basename $CFG)" > "$OUT/run_info.txt"
md5sum $D/*.py install/social_critic/lib/libsocial_critic.so >> "$OUT/run_info.txt"
P="$OUT/replay_params.yaml"; python3 analysis/hw_replay/make_replay_params.py "$CFG" "$P" > /dev/null || exit 1
set -f
python3 ${RAY%% *} --ros-args -p use_sim_time:=true -p camera_topic:=/person_positions_map -p scan_topic:=/scan \
  -p odom_topic:=/odom -p camera_yaw_offset_deg:=2.25 -p camera_fx:=287.31 -p ray_half_width_deg:=6.0 \
  $( [ "${RAY#* }" != "$RAY" ] && echo "${RAY#* }" ) > "$OUT/ray.log" 2>&1 &
R=$!
python3 ${IDN%% *} --ros-args -p use_sim_time:=true -p scan_topic:=/scan -p map_topic:=/map \
  -p coast_enable:=$COAST -p hold_enable:=true $( [ "${IDN#* }" != "$IDN" ] && echo "${IDN#* }" ) > "$OUT/identity.log" 2>&1 &
I=$!
python3 $D/${KF}_orig.py --ros-args -p use_sim_time:=true -p coast_timeout:=1.5 -r /odom:=/odom > "$OUT/kf.log" 2>&1 &
K=$!
set +f
/opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args --params-file "$P" -p use_sim_time:=true \
  -r cmd_vel:=/replay_cmd_vel -r cmd_vel_nav:=/replay_cmd_vel > "$OUT/controller.log" 2>&1 &
C=$!
python3 analysis/hw_replay/path_feeder.py > "$OUT/feeder.log" 2>&1 &
F=$!
python3 analysis/hw_replay/chain_logger.py "$OUT/chain.csv" > "$OUT/logger.log" 2>&1 &
RB=$!
sleep 3
ros2 bag play "$BAG" --clock 50 --qos-profile-overrides-path analysis/hw_replay/tf_static_qos.yaml --remap /turtlebot4/tf:=/tf /turtlebot4/tf_static:=/tf_static \
  /turtlebot4/odom:=/odom /turtlebot4/scan:=/scan /turtlebot4/plan:=/bag_plan /rosout:=/bag_rosout \
  /turtlebot4/goal_pose:=/bag_goal_pose /turtlebot4/map:=/map /camera_ray_clusters:=/bag_ray \
  /person_positions_fused:=/bag_fused /predicted_person_positions:=/bag_pred > "$OUT/play.log" 2>&1 &
B=$!
sleep 3
for i in $(seq 1 40); do
  st=$(ros2 lifecycle get /controller_server 2>/dev/null | awk '{print $1}')
  case "$st" in
    active) echo "controller active after $i checks" > "$OUT/lifecycle.log"; break ;;
    unconfigured) ros2 lifecycle set /controller_server configure > /dev/null 2>&1 ;;
    inactive) ros2 lifecycle set /controller_server activate > /dev/null 2>&1 ;;
  esac
  sleep 0.3
done
wait $B
sleep 2
kill -INT $F $C $K $I $R $RB 2>/dev/null; sleep 3; kill -9 $F $C $K $I $R $RB 2>/dev/null
python3 analysis/hw_replay/compare_decisions.py "$BAG" "$OUT/controller.log" > "$OUT/decisions.txt"
