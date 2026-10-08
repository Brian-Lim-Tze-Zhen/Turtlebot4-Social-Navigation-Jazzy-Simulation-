#!/bin/bash
# run_recjump.sh - replay all extracted robot bags with the recommended + jump-robust config (8 Oct 2026).
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
CFG=config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_recjump_sim.yaml
S=analysis/hw_replay/out/prediction_summary.txt
echo "# recjump started $(date '+%F %T'), critic $(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)" >> $S
for b in $(ls bags/hw_replay_src); do
  timeout 300 analysis/hw_replay/hw_replay.sh "$b" "$CFG" recjump > /dev/null 2>&1
  d="analysis/hw_replay/out/recjump/$b/decisions.txt"
  echo "recjump | $b | replay lane starts $(grep -c 's\* lane rule now follows' $d) | $(sed -n '/^ROBOT/,/^REPLAY/p' $d | grep -c 'lane rule now follows') on the robot | $(head -1 analysis/hw_replay/out/recjump/$b/lifecycle.log)" >> $S
done
echo "# recjump finished $(date '+%F %T')" >> $S
chown -R 1000:1000 analysis/hw_replay
