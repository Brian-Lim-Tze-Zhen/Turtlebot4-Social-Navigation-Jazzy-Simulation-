# Scripts

The entry points stay in the top level of `ros2_ws`, because the scripts here call them:

| Script | What it does |
|---|---|
| `run_headon_F_trial.sh` | one head-on trial: fresh Gazebo, Nav2, perception, walker, goal, recorded bag, teardown |
| `run_conv_F_trial.sh` | one standing-pair or queue trial (`CASE=wide`, `narrow` or `queue`) |
| `run_conv_F_trial_hwsync.sh` | the same pair trial with the robot's perception settings and nodes |
| `run_hwsync_pathside_batch.sh` | a numbered series of head-on trials with the robot's nodes; failed starts are repeated |
| `run_scenario.sh` | the five demo scenarios (see `SCENARIOS.md`) |
| `run_sim.sh` | the whole stack, without a scenario |
| `headon_send_goal.py` | sends the goal and records its result (called by the trial scripts) |

Every script in this folder changes into the workspace itself, so it can be started from anywhere. Inside the container:

```bash
cd /root/thesis_social_navigation_ws
./scripts/experiments/run_rec20ng_groupskip_test.sh headon
```

## experiments/

The batch scripts that recorded the simulation sets of the thesis. Each one lists its bag names, configuration and baseline in its header.

| Script | What it ran |
|---|---|
| `run_rec20ng_groupskip_test.sh` | **final configuration**: head-on, head-on with the walker 0.4 m to the right, wide pair, narrow pair (10 trials each, bags `*_rec20gs_*`) |
| `run_rec20noghost_test.sh` | the same without group skip (bags `*_rec20ng_*`) |
| `run_rec_test.sh`, `run_rec15_test.sh`, `run_rec20_test.sh`, `run_rec20_y04_test.sh`, `run_rec35_test.sh` | side-switch offset 0.15 to 0.35 m (8 and 9 October 2026) |
| `run_sd03_test.sh` | side-decision time 0.3 s |
| `run_hwsync_headon.sh`, `run_hwsync_split.sh`, `run_pathside30.sh` (host) | the robot's changes ported to the simulation (8 October 2026) |
| `run_v4_followup.sh`, `run_v4_nowatch.sh` | critic version 4 (sustained approach) and its two rules separated (7 October 2026) |
| `run_conv_F_gskip_batch.sh` | group skip on the standing pair |
| `run_headon_F_batch.sh` | head-on avoidance attempts (early October 2026) |

## legacy/

Not used for the results of the thesis; kept for the history of the work.

| Item | What it was |
|---|---|
| `run_sim_bringup.sh`, `run_sim_perception.sh`, `run_sim_lidar_occupancy.sh`, `start_lidar_fusion_clean.sh` | the LiDAR-fusion pipeline of August 2026, replaced by the camera-led one |
| `replay_detector_test.sh`, `test_static_streak.sh` | replays of a bag through two versions of the LiDAR person detector |
| `record_trial.sh` | the first bag recorder; the trial scripts now record the bag themselves |
| `analyze_bag_tracks.py`, `newbaseline_results.csv`, `eval_plots/` | early evaluation of tracks and of the August baseline |
| `replay_AB/` | kit to re-run ablation conditions A and B with recorded settings (3 October 2026), replaced by `ablation_ws` |
