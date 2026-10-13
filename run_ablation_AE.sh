#!/bin/bash
# --------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Host-side runner for the re-run of the head-on ablation A to E (Table 7.2 of the thesis).
#   It starts the simulation container on the separate workspace ablation_ws (August state, see
#   ablation_ws/run_ablation_AE_trial.sh) and runs the trials one after the other.
#
#   Usage (on the laptop, not inside a container):
#     ./run_ablation_AE.sh build              build the August critic once (about 2 minutes)
#     ./run_ablation_AE.sh one  <A-E> <name>  one trial with the given bag name (pilot)
#     ./run_ablation_AE.sh batch [n]          n trials per condition (default 6), order A1 B1 C1 D1 E1 A2 ...
#   Before the first run: xhost +local:root   (the August YOLO node opens a small window)
#   Reads:  ablation_ws/
#   Writes: ablation_ws/bags/<condition>_rr_tNN/, ablation_ws/logs/, ablation_log_20261013.txt
#   A trial that fails at start-up (no bag or no goal result) is logged and repeated as _a2, _a3.
# --------------------------------------------------------------------------------------------------
set -e
ROOT=/home/brian/thesis_social_navigation
WS=$ROOT/ablation_ws
IMAGE=thesis-social-nav-snap:20261010
LOG=$ROOT/ablation_log_20261013.txt
MODE=${1:-}

in_container() {   # in_container <command string>
  docker run --rm --name ablation_rerun --privileged --shm-size 4g \
    -e DISPLAY=${DISPLAY:-:0} -e QT_X11_NO_MITSHM=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e LIBGL_ALWAYS_SOFTWARE=0 -e MESA_LOADER_DRIVER_OVERRIDE=iris -e LIBGL_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri --device /dev/dri \
    -v "$HOME/.gz:/root/.gz" -v "$WS:/root/thesis_social_navigation_ws" \
    $IMAGE bash -c "$1"
}
free_mb() { df --output=avail -m /home | tail -1 | tr -d ' '; }

declare -A PREFIX=([A]=a_nolayer [B]=b_critweight20 [C]=c_critweight10 [D]=d_critweight10_socialcritic_on [E]=e_critweight20_socialcritic_on)

run_one() {   # run_one <condition> <bag name>; returns 0 if a bag and a goal result exist
  local c=$1 name=$2
  [ "$(free_mb)" -lt 1500 ] && { echo "Only $(free_mb) MB free - stopping." >&2; exit 1; }
  in_container "cd /root/thesis_social_navigation_ws && ./run_ablation_AE_trial.sh $c $name" > "$WS/logs/$name.console" 2>&1 || true
  local bagfiles; bagfiles=$(ls "$WS/bags/$name/data/"*.mcap 2>/dev/null | wc -l)
  local goal; goal=$(tail -1 "$WS/bags/$name/goal_result.txt" 2>/dev/null || echo none)
  echo "$(date '+%F %T') $name: bag files $bagfiles, goal: $goal, $(grep -h 'Condition check' "$WS/logs/$name.console" | tail -1)" | tee -a "$LOG"
  [ "$bagfiles" -ge 1 ] && [ "$goal" != none ]
}

case "$MODE" in
  build)
    in_container "cd /root/thesis_social_navigation_ws && source /opt/ros/jazzy/setup.bash && mkdir -p logs && colcon build --base-paths src --packages-select social_critic > logs/critic_build.log 2>&1; tail -3 logs/critic_build.log; md5sum install/social_critic/lib/libsocial_critic.so"
    ;;
  one)
    run_one "$2" "$3" || echo "start-up failure - see $WS/logs/$3.console"
    tail -5 "$WS/logs/$3.console"
    ;;
  batch)
    N=${2:-6}
    for t in $(seq 1 "$N"); do
      for c in A B C D E; do
        base=$(printf "%s_rr_t%02d" "${PREFIX[$c]}" "$t")
        ls -d "$WS/bags/$base"* > /dev/null 2>&1 && { echo "$base exists - skipped"; continue; }
        name=$base
        for a in 1 2 3; do
          [ "$a" -gt 1 ] && name="${base}_a$a"
          if run_one "$c" "$name"; then break; fi
          echo "$(date '+%F %T') $name: start-up failure (tail: $(tail -2 "$WS/logs/$name.console" | tr '\n' ' ')) - folder removed, repeating" | tee -a "$LOG"
          docker run --rm -v "$WS:/w" $IMAGE rm -rf "/w/bags/$name"
        done
      done
    done
    ;;
  *) sed -n 2,16p "$0"; exit 1 ;;
esac
