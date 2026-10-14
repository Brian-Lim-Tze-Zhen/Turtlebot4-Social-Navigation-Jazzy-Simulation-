#!/bin/bash
# run_rec20_y04_test.sh - rec20 config, walker 0.4 m to the robot's right (PERSON_Y -0.4), 10 head-on trials (9 Oct 2026).
# Checks the other half of the side rule: a walker clearly on the robot's right must get "robot keeps LEFT".
# Run inside the container. Bags headon_rec20_y-0.4_trial01-10. Centred counterpart: run_rec20_test.sh.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
S="$WS/logs/rec20_summary.txt"
H=$(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)
CFG="$WS/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec20_sim.yaml"
export PERSON_Y=-0.4
echo "# rec20 y-0.4 test started $(date '+%F %T'), critic $H, PERSON_Y $PERSON_Y" >> "$S"
SIDE_REF="robot-goal line" CFG_OVERRIDE="$CFG" PREFIX=headon_rec20_y-0.4_trial PAD2=1 ./run_hwsync_pathside_batch.sh 1 10
grep "headon_rec20_y-0.4_trial" logs/hwsync_pathside_batch_summary.txt >> "$S"
echo "# rec20 y-0.4 test finished $(date '+%F %T')" >> "$S"
