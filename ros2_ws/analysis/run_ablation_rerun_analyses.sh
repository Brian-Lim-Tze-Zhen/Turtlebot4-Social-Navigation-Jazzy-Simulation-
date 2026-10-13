#!/bin/bash
# Runs the Chapter 4 / Section 7.2 analyses of the A-E ablation on the re-run bags (*_rr_t*), in the jazzy image.
# Writes <script>_rerun.txt next to each earlier output; the outputs of the first run are not touched.
# Usage (host): analysis/run_ablation_rerun_analyses.sh
WS=$HOME/thesis_social_navigation/ros2_ws
docker run --rm -v "$WS:/ws" thesis-social-nav-jazzy:latest bash -c '
source /opt/ros/jazzy/setup.bash; cd /ws/analysis
RR=$(ls -d /ws/bags/[a-e]_*_rr_t0* | sort)
D=$(date "+%-d %b %Y")
{ echo "# headon_ablation_onset.py on the 30 re-run bags, computed $D in the jazzy container (same definitions as headon_ablation_onset.txt)"; python3 headon_ablation_onset.py $RR; } > headon_ablation_onset_rerun.txt 2>&1
{ echo "# headon_onset_profile.py on the 30 re-run bags, computed $D in the jazzy container (same definitions as headon_onset_profile.txt)"; python3 headon_onset_profile.py $RR; } > headon_onset_profile_rerun.txt 2>&1
{ echo "# headon_cloud_extent.py on the re-run bags of B to E, computed $D (recorded /predicted_person_cloud)"; python3 headon_cloud_extent.py $(for b in $RR; do case $b in */a_*) ;; *) echo $b/data;; esac; done); } > headon_cloud_extent_rerun.txt 2>&1
{ echo "# ablation_rollout_length.py on the 30 re-run bags, computed $D"; python3 ablation_rollout_length.py $RR; } > ablation_rollout_length_rerun.txt 2>&1
{ echo "Lateral rate of the robot, re-run bags of B and E, computed $D. Script: analysis/headon_lateral_rate.py rr"; python3 headon_lateral_rate.py rr; } > headon_lateral_rate_rerun.txt 2>&1
{ echo "Kalman predictor in the A-E re-run bags, computed $D. Script: analysis/headon_ablation_kf_check.py bags/[a-e]_*_rr_t0*"; python3 headon_ablation_kf_check.py $RR; } > headon_ablation_kf_check_rerun.txt 2>&1
echo done'
