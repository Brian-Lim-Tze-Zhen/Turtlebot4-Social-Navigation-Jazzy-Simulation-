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

Every script in `experiments/` changes into the workspace itself, so it can be started from anywhere. Inside the container:

```bash
cd /root/thesis_social_navigation_ws
./scripts/experiments/run_rec20ng_groupskip_test.sh headon
```

## experiments/

The batch scripts of the final configuration (Sections 7.3, 7.4 and 7.6 of the thesis):

| Script | What it runs |
|---|---|
| `run_rec20ng_groupskip_test.sh` | **final configuration**: head-on, head-on with the walker 0.4 m to the right, wide pair, narrow pair (10 trials each, bags `*_rec20gs_*`) |
| `run_rec20noghost_test.sh` | the same without group skip (bags `*_rec20ng_*`), compared with the robot in Section 7.6 |

The comparison sets of the thesis (baseline, lane rules, simulation version) are single trials of `run_headon_F_trial.sh` and `run_conv_F_trial.sh` with another `CFG`; the commands are in `HOW_TO_RUN.md`, section 4. The batch scripts of the tuning steps (side-switch offset, side-decision time, the robot's settings ported to the simulation) and the older tools were removed on 14 October 2026; they remain in the git history.
