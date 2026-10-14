#!/bin/bash
# run_rec35_test.sh - recommended config with side_switch_offset 0.35 (8 Oct 2026). Run inside the container.
#   1. head-on corridor, 10 trials, ..._hwsync_rec35_sim.yaml  -> bags headon_rec35_trial01-10
#      (run_hwsync_pathside_batch.sh: robot nodes for *hwsync*, any outcome counts, failed starts repeated)
#   2. pair regression, same config, narrow and wide, 6 trials each  -> conv_F_rec35_<case>_trialN
#      (run_conv_F_trial.sh, perception launch defaults as in conv_F_v4; repeated only for a failed start:
#      no final goal status, wrong critic hash or a zone map with fewer than 20 messages)
# Baselines: headon_pathside30_trial01-13 (side_decision_s 0.6) and conv_F_v4_<case>_trial1-6.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
S="$WS/logs/rec35_summary.txt"
H=$(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)
CFG="$WS/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec35_sim.yaml"
echo "# rec35 test started $(date '+%F %T'), critic $H" >> "$S"
SIDE_REF="robot-goal line" CFG_OVERRIDE="$CFG" PREFIX=headon_rec35_trial PAD2=1 ./run_hwsync_pathside_batch.sh 1 10
grep "headon_rec35_trial" logs/hwsync_pathside_batch_summary.txt >> "$S"
for c in narrow wide; do
  for i in 1 2 3 4 5 6; do
    tries=0; ok=false
    while [ $tries -lt 3 ] && [ "$ok" != true ]; do
      tries=$((tries + 1)); name=conv_F_rec35_${c}_trial$i; [ $tries -gt 1 ] && name="${name}_a${tries}"
      env CFG="$CFG" CASE=$c ./run_conv_F_trial.sh "$name" > "logs/$name.out" 2>&1; rc=$?
      goal=$(grep -i "finished" "bags/$name/goal_result.txt" 2>/dev/null | tail -1 | grep -o '[A-Z]*$')
      md5=$(cut -c1-8 "bags/$name/critic_so_md5.txt" 2>/dev/null | head -1)
      lanes=$(grep -c "lane rule now follows" "logs/$name/nav2.log" 2>/dev/null)
      zone=$(ros2 bag info "bags/$name" 2>/dev/null | grep -o 'social_zone_map.*Count: [0-9]*' | grep -o '[0-9]*$')
      if [ -n "$goal" ] && [ "$md5" = "$H" ] && [ "${zone:-0}" -ge 20 ]; then ok=true; v=COUNTED; else v="REPEAT (start, rc=$rc)"; fi
      echo "$name | $v | goal ${goal:-none} | critic ${md5:-none} | lane lines ${lanes:-0} | zone ${zone:-0}" >> "$S"
      [ "$ok" = true ] || { pkill -f "gz sim"; sleep 5; }
    done
  done
done
echo "# rec35 test finished $(date '+%F %T')" >> "$S"
