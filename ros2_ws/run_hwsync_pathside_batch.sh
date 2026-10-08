#!/bin/bash
# run_hwsync_pathside_batch.sh <first> <last> - head-on corridor trials with the synced config (8 Oct 2026).
# PREFIX=<bag name prefix> (default headon_hwsync_pathside_corr_trial); trial numbers get two digits when PAD2=1.
# Run inside the container. Config ..._hwsync_pathside_sim.yaml (= robot ..._approach_tf05_pathside_hw.yaml);
# run_headon_F_trial.sh picks the noghost predictor, the front-cone blocked node and the robot retry node for it.
# Bags <PREFIX>N. A trial counts once its goal has a final status (any outcome) and the bag carries the current
# critic hash, the critic measured the side against the path, the noghost predictor started and the robot
# beep/retry nodes ran; otherwise it is repeated (up to 3 attempts, _a2, _a3). Failed starts are only ever
# repeated for a failed start or check, never for their outcome. One line per attempt in
# logs/hwsync_pathside_batch_summary.txt.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
S="$WS/logs/hwsync_pathside_batch_summary.txt"
H=$(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)
CFG_PS="${CFG_OVERRIDE:-$WS/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_pathside_sim.yaml}"   # CFG_OVERRIDE: another *hwsync* config
echo "# batch ${PREFIX:-headon_hwsync_pathside_corr_trial} $1-$2 started $(date '+%F %T'), critic $H, config $(basename "$CFG_PS")" >> "$S"
for i in $(seq "$1" "$2"); do
  tries=0; ok=false
  while [ $tries -lt 3 ] && [ "$ok" != true ]; do
    tries=$((tries + 1)); n=$i; [ "${PAD2:-0}" = 1 ] && n=$(printf %02d "$i"); name=${PREFIX:-headon_hwsync_pathside_corr_trial}$n; [ $tries -gt 1 ] && name="${name}_a${tries}"
    env CFG="$CFG_PS" ./run_headon_F_trial.sh "$name" > "logs/$name.out" 2>&1; rc=$?
    goal=$(grep -i "finished" "bags/$name/goal_result.txt" 2>/dev/null | tail -1 | grep -o '[A-Z]*$')
    md5=$(cut -c1-8 "bags/$name/critic_so_md5.txt" 2>/dev/null | head -1)
    side=$(grep -c "pass side measured against the ${SIDE_REF:-start of the global path}" "logs/$name/nav2.log" 2>/dev/null)   # SIDE_REF="robot-goal line" for side_ref_goal configs
    ng=$(grep -c "NOGHOST: a track" "logs/$name/perception.log" 2>/dev/null)
    nodes=$(grep -c "blocked_person_beep_node_front_sim.py, beep_retry_node_hwsync_sim.py" "bags/$name/run_notes.txt" 2>/dev/null)
    # The OUTCOME never causes a repeat: a trial counts once its goal has a final status (SUCCEEDED,
    # ABORTED, ...) and the set-up checks pass. Only a failed start (no final status) or a wrong set-up is repeated.
    if [ -n "$goal" ] && [ "$md5" = "$H" ] && [ "${side:-0}" -gt 0 ] \
       && [ "${ng:-0}" -gt 0 ] && [ "${nodes:-0}" -gt 0 ]; then ok=true; v="COUNTED"; else v="REPEAT (start/set-up, rc=$rc)"; fi
    echo "$name | $v | goal ${goal:-none} | critic ${md5:-none} | path side ${side:-0} | noghost ${ng:-0} | robot nodes ${nodes:-0}" >> "$S"
    [ "$ok" = true ] || { pkill -f "gz sim"; sleep 5; }
  done
done
echo "# batch ${PREFIX:-headon_hwsync_pathside_corr_trial} $1-$2 finished $(date '+%F %T')" >> "$S"
