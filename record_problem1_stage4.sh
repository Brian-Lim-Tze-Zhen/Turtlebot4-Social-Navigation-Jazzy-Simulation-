#!/bin/bash
# --------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Runs the narrow standing-pair scenario with the lane rules of stage 4 (the configuration that
#   started a lane for a standing member, "Problem 1"), with the Gazebo window and RViz open, so
#   that the run can be screen-recorded for the progress meeting.
#   It uses the reproduction workspace (repro_ws), exactly as conv_F_stage4_narrow_trial1-6 of
#   10 October 2026: critic package of commit b06d253, the recorded Nav2 file of trial 3, and the
#   perception nodes and trial script already in repro_ws (same md5 as the recording).
#
#   Usage (on the host, not inside a container):
#     ./record_problem1_stage4.sh           Gazebo + RViz open (for the video)
#     ./record_problem1_stage4.sh test      same run without windows (check only)
#     ./record_problem1_stage4.sh restore   put back the critic that was in repro_ws before
#   Before the first run with windows:  xhost +local:root
#   Writes: repro_ws/bags/video_stage4_narrow_<time>/ (about 35 MB), logs in repro_ws/logs/
# --------------------------------------------------------------------------------------------------
set -e
ROOT=/home/brian/thesis_social_navigation
WS=$ROOT/repro_ws
MODE=${1:-video}
IMAGE=thesis-social-nav-snap:20261010
STAMP=$(date +%Y%m%d_%H%M%S)
NAME=video_stage4_narrow_$STAMP

if docker ps --format '{{.Names}}' | grep -q '^video_stage4$'; then
  echo "A container named video_stage4 is still running. Stop it first: docker stop video_stage4" >&2; exit 1
fi
FREE=$(df --output=avail -m /home | tail -1)
[ "$FREE" -lt 600 ] && { echo "Only $FREE MB free on the disk - free some space first (the screen recording needs room too)." >&2; exit 1; }

# stage-4 critic sources and the recorded Nav2 file, taken read-only from git and from the bag
mkdir -p "$WS/stage4_restore"
if [ ! -d "$WS/stage4_restore/social_critic" ]; then
  git -C "$ROOT" archive b06d253 ros2_ws/src/social_critic | tar -x -C "$WS/stage4_restore" --strip-components=2
fi
cp "$ROOT/ros2_ws/bags/conv_F_stage4_narrow_trial3/config_used.yaml" "$WS/config/social_nav2_stage4_recorded_sim.yaml"

cat > "$WS/video_stage4_inner.sh" <<'INNER'
#!/bin/bash
set -e
MODE=$1; NAME=$2
WS=/root/thesis_social_navigation_ws
cd $WS
source /opt/ros/jazzy/setup.bash
mkdir -p critic_backups
if [ "$MODE" = restore ]; then
  B=$(ls -d critic_backups/social_critic_before_stage4_* 2>/dev/null | head -1)
  [ -n "$B" ] || { echo "nothing to restore"; exit 0; }
  mv src/social_critic critic_backups/social_critic_stage4_used_$(date +%s)
  mv "$B" src/social_critic
  touch critic_backups/COLCON_IGNORE stage4_restore/COLCON_IGNORE
  colcon build --base-paths src --packages-select social_critic > logs/restore_critic_build.log 2>&1 || { tail -30 logs/restore_critic_build.log; exit 1; }
  echo "critic restored: $(md5sum install/social_critic/lib/libsocial_critic.so)"; exit 0
fi
touch critic_backups/COLCON_IGNORE stage4_restore/COLCON_IGNORE
if ! cmp -s src/social_critic/src/social_critic.cpp stage4_restore/social_critic/src/social_critic.cpp; then
  mv src/social_critic critic_backups/social_critic_before_stage4_$(date +%Y%m%d_%H%M%S)
  cp -r stage4_restore/social_critic src/social_critic
fi
echo "[video] building the stage-4 critic (about 2 minutes the first time)..."
colcon build --base-paths src --packages-select social_critic > logs/stage4_critic_build.log 2>&1 || { tail -30 logs/stage4_critic_build.log; exit 1; }
echo "[video] critic: $(md5sum install/social_critic/lib/libsocial_critic.so)"
export CASE=narrow CFG=$WS/config/social_nav2_stage4_recorded_sim.yaml GOAL_TIMEOUT=240
if [ "$MODE" = video ]; then export SHOW_RVIZ=true HEADLESS=false; fi
./run_conv_F_trial.sh "$NAME"
python3 /ana/analyse_F_narrow.py bags/$NAME 2>/dev/null | grep -E "goal reached|max\|y\||min centre|gap window" || true
grep -c "lane" logs/$NAME/nav2.log 2>/dev/null | sed 's/^/[video] lines mentioning a lane in nav2.log: /' || true
INNER
chmod +x "$WS/video_stage4_inner.sh"

docker run --rm --name video_stage4 --privileged --shm-size 4g \
  -e DISPLAY=${DISPLAY:-:0} -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root -e ROS_DOMAIN_ID=88 \
  -e LIBGL_ALWAYS_SOFTWARE=0 -e MESA_LOADER_DRIVER_OVERRIDE=iris -e LIBGL_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri --device /dev/dri \
  -v "$HOME/.gz:/root/.gz" -v "$WS:/root/thesis_social_navigation_ws" -v "$ROOT/ros2_ws/analysis:/ana:ro" \
  $IMAGE bash -c "/root/thesis_social_navigation_ws/video_stage4_inner.sh $MODE $NAME"
echo "$(date '+%F %T') record_problem1_stage4.sh $MODE: $NAME (stage-4 critic b06d253 + recorded Nav2 file of conv_F_stage4_narrow_trial3; for the progress-meeting video, not a thesis trial)" >> "$ROOT/repro_log_20261010.txt"
