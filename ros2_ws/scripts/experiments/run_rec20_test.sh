#!/bin/bash
# run_rec20_test.sh - rec15 config with side_switch_offset 0.20 (9 Oct 2026). Run inside the container.
#   head-on corridor, 10 trials, ..._hwsync_rec20_sim.yaml -> bags headon_rec20_trial01-10
#   (run_hwsync_pathside_batch.sh: robot nodes for *hwsync*, any outcome counts, failed starts repeated)
# Baseline: headon_rec15_trial01-10 (run_rec15_test.sh, 8 Oct): 8 right / 2 left, 0.746 +/- 0.043 m.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
S="$WS/logs/rec20_summary.txt"
H=$(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)
CFG="$WS/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec20_sim.yaml"
echo "# rec20 test started $(date '+%F %T'), critic $H" >> "$S"
SIDE_REF="robot-goal line" CFG_OVERRIDE="$CFG" PREFIX=headon_rec20_trial PAD2=1 ./run_hwsync_pathside_batch.sh 1 10
grep "headon_rec20_trial" logs/hwsync_pathside_batch_summary.txt >> "$S"
echo "# rec20 test finished $(date '+%F %T')" >> "$S"
