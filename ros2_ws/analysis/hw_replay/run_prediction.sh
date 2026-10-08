#!/bin/bash
# run_prediction.sh - replay the robot bags with the candidate config, and the pair bags also with the decided
# config (8 Oct 2026). Inside the simulation container. One line per replay in analysis/hw_replay/out/prediction_summary.txt.
WS=/root/thesis_social_navigation_ws; cd "$WS" || exit 1
CAND=config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_goalside_aw05m085ov45w08hold_sim.yaml
DEC=config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_pathside_sim.yaml
S=analysis/hw_replay/out/prediction_summary.txt
echo "# prediction started $(date '+%F %T')" >> $S
run() {   # run <bag> <cfg> <tag>
  timeout 300 analysis/hw_replay/hw_replay.sh "$1" "$2" "$3" > /dev/null 2>&1
  d="analysis/hw_replay/out/$3/$1/decisions.txt"
  lanes=$(grep -c "s\* lane rule now follows" "$d" 2>/dev/null)
  echo "$3 | $1 | replay lane starts ${lanes:-0} | $(grep -c 'lane rule now follows' <(sed -n '/^ROBOT/,/^REPLAY/p' "$d")) on the robot | $(cat analysis/hw_replay/out/$3/$1/lifecycle.log | head -1)" >> $S
}
for b in $(ls bags/hw_replay_src); do run "$b" "$CAND" candidate; done
for b in $(ls bags/hw_replay_src | grep ^pair); do run "$b" "$DEC" decided; done
echo "# prediction finished $(date '+%F %T')" >> $S
chown -R 1000:1000 analysis/hw_replay
