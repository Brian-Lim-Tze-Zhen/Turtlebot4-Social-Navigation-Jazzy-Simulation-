#!/bin/bash
# run_rec20noghost_test.sh [headon|headon_y04|wide|narrow ...] - simulation runs of the robot's final configuration (9 Oct 2026).
# Config ..._hwsync_rec20_noghost_sim.yaml (= robot ..._tf05_rec20_noghost_hw.yaml: 57 critic parameters equal), critic lib with
# the noghost parameters (894884fa or later), noghost predictor, at-goal stop-and-beep hold node, robot retry node.
#   headon      10 centred head-on trials            -> headon_rec20ng_trial01-10      (run_hwsync_pathside_batch.sh)
#   headon_y04  10 head-on, walker 0.4 m to the right -> headon_rec20ng_y-0.4_trial01-10
#   wide        10 standing pair, wide corridor      -> conv_F_rec20ng_wide_trialN     (run_conv_F_trial_hwsync.sh)
#   narrow      10 standing pair, narrow corridor    -> conv_F_rec20ng_narrow_trialN
# Robot sets of 9 Oct: headon_rec20noghost_run01-10, pairwide_rec20noghost_run01-10, pairnarrow_rec20noghost_run01-10.
# Run inside the container. Default: all four in that order.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
S="$WS/logs/rec20noghost_summary.txt"
H=$(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)
CFG="$WS/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec20_noghost_sim.yaml"
export BLOCKED_BEEP_SCRIPT=blocked_person_beep_node_front_atgoal_sim.py
for part in ${@:-headon headon_y04 wide narrow}; do
  echo "# $part started $(date '+%F %T'), critic $H" >> "$S"
  case $part in
    headon)     SIDE_REF="robot-goal line" CFG_OVERRIDE="$CFG" PREFIX=headon_rec20ng_trial PAD2=1 ./run_hwsync_pathside_batch.sh 1 10
                grep "headon_rec20ng_trial" logs/hwsync_pathside_batch_summary.txt >> "$S" ;;
    headon_y04) PERSON_Y=-0.4 SIDE_REF="robot-goal line" CFG_OVERRIDE="$CFG" PREFIX=headon_rec20ng_y-0.4_trial PAD2=1 ./run_hwsync_pathside_batch.sh 1 10
                grep "headon_rec20ng_y-0.4_trial" logs/hwsync_pathside_batch_summary.txt >> "$S" ;;
    wide|narrow)
      for i in $(seq 1 10); do
        tries=0; ok=false
        while [ $tries -lt 3 ] && [ "$ok" != true ]; do
          tries=$((tries + 1)); name=conv_F_rec20ng_${part}_trial$(printf %02d $i); [ $tries -gt 1 ] && name="${name}_a${tries}"
          env CFG="$CFG" CASE=$part ./run_conv_F_trial_hwsync.sh "$name" > "logs/$name.out" 2>&1; rc=$?
          goal=$(grep -i "finished" "bags/$name/goal_result.txt" 2>/dev/null | tail -1 | grep -o '[A-Z]*$')
          md5=$(cut -c1-8 "bags/$name/critic_so_md5.txt" 2>/dev/null | head -1)
          ng=$(grep -c "noghost: coasted targets do not start" "logs/$name/nav2.log" 2>/dev/null)
          ag=$(grep -c "\[atgoal\]" "logs/$name/blocked_beep.log" 2>/dev/null)
          zone=$(ros2 bag info "bags/$name" 2>/dev/null | grep -o 'social_zone_map.*Count: [0-9]*' | grep -o '[0-9]*$')
          if [ -n "$goal" ] && [ "$md5" = "$H" ] && [ "${zone:-0}" -ge 20 ] && [ "${ng:-0}" -ge 1 ] && [ "${ag:-0}" -ge 1 ]; then ok=true; v=COUNTED; else v="REPEAT (start/set-up, rc=$rc)"; fi
          echo "$name | $v | goal ${goal:-none} | critic ${md5:-none} | noghost ${ng:-0} | atgoal ${ag:-0} | zone ${zone:-0}" >> "$S"
          [ "$ok" = true ] || { pkill -f "gz sim"; sleep 5; }
        done
      done ;;
  esac
  echo "# $part finished $(date '+%F %T')" >> "$S"
done
