#!/bin/bash
# run_pchain.sh - every extracted robot bag through the perception chain + critic, original nodes and fix copies
# (8 Oct 2026). Critic config: the recommended one. Summary: analysis/hw_replay/out/pchain_summary.txt
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
CFG=config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec_sim.yaml
S=analysis/hw_replay/out/pchain_summary.txt
echo "# ${TAGP:-pchain} started $(date '+%F %T'), critic $(md5sum install/social_critic/lib/libsocial_critic.so | cut -c1-8)" >> $S
for b in $(ls bags/hw_replay_src_id); do
  case $b in headon_lanerule*|*fullstack*) P=lane ;; headon_v4*) P=noghost ;; *) P=base ;; esac
  for m in orig fix; do
    timeout 320 analysis/hw_replay/hw_perception_replay.sh "$b" $P $m "$CFG" ${TAGP:-pchain}_$m > /dev/null 2>&1
    d=analysis/hw_replay/out/${TAGP:-pchain}_$m/$b
    lanes=$(grep -c "s\* lane rule now follows" $d/decisions.txt 2>/dev/null)
    splits=$(grep -c "different object" $d/identity.log 2>/dev/null)
    rel=$(grep -c "motion-release" $d/ray.log 2>/dev/null)
    echo "${TAGP:-pchain} $m | $b | profile $P | lane starts ${lanes:-0} | still splits ${splits:-0} | motion releases ${rel:-0} | $(head -1 $d/lifecycle.log)" >> $S
  done
done
echo "# ${TAGP:-pchain} finished $(date '+%F %T')" >> $S
chown -R 1000:1000 analysis/hw_replay
