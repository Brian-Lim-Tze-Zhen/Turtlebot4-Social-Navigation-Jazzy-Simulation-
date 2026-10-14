#!/bin/bash
# run_pathside30.sh - host side (8 Oct 2026): 30 head-on trials with the synced config in three batches of 10,
# bags headon_pathside30_trial01-30, with the CPU sampler recording each batch on the host.
cd "$(dirname "$0")/../.." || exit 1   # ros2_ws (this script is in scripts/experiments/)
for b in "1 10" "11 20" "21 30"; do
  set -- $b
  python3 analysis/cpu_sampler.py "analysis/cpu/pathside30_batch_$1_$2_cpu.csv" & SP=$!
  docker exec thesis_social_nav bash -c "cd /root/thesis_social_navigation_ws && PREFIX=headon_pathside30_trial PAD2=1 ./run_hwsync_pathside_batch.sh $1 $2 >> logs/pathside30_batch.out 2>&1"
  kill -TERM $SP; wait $SP
done
