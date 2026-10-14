# How to run the simulation

Start here. This guide covers four things:

1. what each folder holds;
2. how to set up the container once;
3. how to run a demo;
4. how to repeat every simulation experiment of the thesis, and where its result is written.

[docs/DEVELOPMENT_NOTES.md](docs/DEVELOPMENT_NOTES.md) keeps the detailed development notes up to 4 October 2026, and `SCENARIOS.md` describes the five demos for a visitor.

---

## 1. Folder map

Rows marked *(local)* are on the author's laptop and the thesis data carrier only, not on GitHub (recordings are too large for git).

| Folder or file | What it is | Rule |
|---|---|---|
| `docker/` | Dockerfile and `docker-compose.yaml` of the simulation image (ROS 2 Jazzy, Nav2, Gazebo Harmonic, TurtleBot 4) | needed to run anything |
| `ros2_ws/` | the workspace; mounted in the container as `/root/thesis_social_navigation_ws` | **do not move or rename anything inside** (scripts and the thesis use these paths) |
| `ros2_ws/src/` | ROS 2 packages: `social_perception` (detector, tracker, predictor, person cloud, group detector) and `social_critic` (MPPI SocialCritic) | code |
| `ros2_ws/config/` | Nav2 parameter files, one per configuration tested | code |
| `ros2_ws/launch/`, `maps/`, `behavior_trees/`, `simulation_models/` | launch files, maps, behaviour trees, Gazebo worlds and the person model | code |
| `ros2_ws/run_*.sh` | trial and batch scripts (section 4) | code |
| `ros2_ws/analysis/` | analysis scripts and their saved reports (`*_report.txt`) | thesis evidence |
| `ros2_ws/bags/` *(local)* | every recorded trial (about 15 GB). Each bag folder also holds `run_notes.txt` (all settings), `config_used.yaml`, `trial_script_used.sh` and copies of the nodes that ran | thesis evidence |
| `ros2_ws/logs/` *(local)* | Nav2 logs and console output per trial; batch summaries | thesis evidence |
| `ablation_ws/` | separate workspace in the state of August 2026 for the ablation A to E, with its host runner `run_ablation_AE.sh` (its bags and logs *(local)*) | thesis evidence |
| `repro_ws/`, `repro_results/` *(local)* | reproduction workspace and results of 10 to 11 October 2026 (older critic versions rebuilt from git); the pair controls were run here | thesis evidence |
| `repro_log_20261010.txt` *(local)* | log of the reproduction of 10 and 11 October 2026 (every repeated or discarded attempt with its reason); the log of the ablation re-run is `ros2_ws/analysis/headon_ablation_rerun_log.txt` | thesis evidence |
| `HANDOFF.md`, `HANDOFF_next_chat.md`, `notes/` *(local)* | development notes (`HANDOFF_next_chat.md` is cited in Appendix E of the thesis) | data carrier |
| `archive/` *(local)* | unused leftovers moved here on 14 October 2026 (August backups, `.bak` copies of scripts); not used by anything | can be deleted |

---

## 2. Set up once

You need Linux with Docker and Docker Compose, an X11 display, and Intel/Mesa graphics. For other graphics, change the environment variables in `docker/docker-compose.yaml`.

```bash
cd ~/thesis_social_navigation/docker
docker compose build            # image thesis-social-nav-jazzy
docker compose up -d            # container thesis_social_nav
xhost +local:root               # lets Gazebo, RViz and the YOLO window open

docker exec -it thesis_social_nav bash
cd /root/thesis_social_navigation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
exit
```

The YOLO weights are downloaded on first use. After a reboot, run `docker start thesis_social_nav` and `xhost +local:root` again.

**Every command in sections 3 and 4 runs inside the container**, in `/root/thesis_social_navigation_ws`, unless it says *host*:

```bash
docker exec -it thesis_social_nav bash
cd /root/thesis_social_navigation_ws && source install/setup.bash
```

---

## 3. Demos (one command each, 2 to 4 minutes)

```bash
./run_scenario.sh headon                # one walker in the 2.5 m corridor
./run_scenario.sh headon_blocked        # two people block the corridor: stop and beep
./run_scenario.sh conversation_wide     # standing pair in open space: go around
./run_scenario.sh conversation_narrow   # same pair in a 2.0 m corridor: pass between
./run_scenario.sh queue                 # four people in a line: go around the end
```

Gazebo and RViz open by themselves, and the initial pose and goal are set by the script. Each run ends with a short summary. See `SCENARIOS.md` for what to look for.

To watch a single trial with RViz, add `SHOW_RVIZ=true HEADLESS=false` in front of any `run_headon_F_trial.sh` or `run_conv_F_trial.sh` command.

---

## 4. The experiments of the thesis

### How a trial works

- **Head-on trial.** `run_headon_F_trial.sh <bag_name>` runs one head-on trial in the 2.5 m corridor. It starts a fresh Gazebo, undocks, sends the walker and the goal, records the bag and shuts everything down.
- **Pair trial.** `run_conv_F_trial.sh <bag_name>` runs one standing-pair trial (`CASE=wide` or `CASE=narrow`). `run_conv_F_trial_hwsync.sh` does the same with the robot's perception settings and nodes.
- **Configuration.** The Nav2 configuration is set with `CFG=config/<file>.yaml`. The trial script picks the matching perception settings from the file name: `*hwreq*` gives the body disk and ray coast, and `*hwsync*` gives the robot's predictor.
- **Bag names.** A bag name that already exists is refused. A failed start is repeated with `_a2`, `_a3`.
- **Exact settings of an old run.** Read `bags/<name>/run_notes.txt` and `bags/<name>/config_used.yaml`. The script that ran is saved as `trial_script_used.sh`.

All configuration names below are in `config/` and start with `social_nav2_`. The prefix is left out of the table.

### 4.1 Ablation A to E (head-on, open space; Chapter 4 and Section 7.2)

This experiment runs in its own workspace `ablation_ws`. The ablation compares five conditions A to E of the August 2026 system, so it must run the August code, not the final one in `ros2_ws`: the Nav2 parameter files, SocialCritic package, world, map, behaviour tree, YOLO detector and LiDAR fusion of commit `eb3f9ac` (15 August 2026), plus the person cloud node, predictor and walker script as saved in the August bags (including a cloud ellipse width that was never committed). Building that old critic in `ros2_ws` would overwrite the final one, hence the separate workspace. Compared with August, each trial now also saves the launched parameter file, the live controller parameters, the Nav2 log and the process list. The author ran it with `thesis-social-nav-snap:20261010`, a `docker commit` of the container of 10 October 2026. That image differs from the one built from `docker/` only in caches and in the worlds and launch files, which the trial script copies in itself, so without it use the public image:

```bash
IMAGE=thesis-social-nav-jazzy ablation_ws/run_ablation_AE.sh build
IMAGE=thesis-social-nav-jazzy ablation_ws/run_ablation_AE.sh batch 6
```

The author's own commands were:

```bash
# host, in ~/thesis_social_navigation
xhost +local:root
ablation_ws/run_ablation_AE.sh build          # build the August critic once (about 2 min)
ablation_ws/run_ablation_AE.sh batch 6        # 6 trials x conditions A-E, order A1 B1 C1 D1 E1 A2 ...
ablation_ws/run_ablation_AE.sh one C c_test   # a single pilot trial
cp -r ablation_ws/bags/[a-e]_*_rr_t* ros2_ws/bags/     # the analyses read the bags from ros2_ws/bags
ros2_ws/analysis/run_ablation_rerun_analyses.sh   # all Chapter 4 / Section 7.2 analyses
```

| Output | Location |
|---|---|
| bags | `ablation_ws/bags/<condition>_rr_tNN/` |
| trial log | `ablation_ws/ablation_log_20261013.txt` (the thesis copy is `ros2_ws/analysis/headon_ablation_rerun_log.txt`) |
| report | `ros2_ws/analysis/headon_ablation_rerun_report.txt` and the other `*_rerun.txt` |

### 4.2 Head-on corridor (Chapter 5, Section 7.3)

| Thesis set | Configuration (`CFG=config/social_nav2_…`) | Bags | Report in `analysis/` |
|---|---|---|---|
| Baseline (stage 3) | `ablation_F_socialzone_sim.yaml` (default) | `headon_F_corridor_trial1-10` | `headon_F_corridor_n10_report.txt` |
| Lane rules (stage 4) | `headon_F_hwreq_block_sim.yaml` | `headon_hwreq_z5_corr_trial1-10` | `headon_hwreq_z5_corr_n10_report.txt` |
| Simulation version | `headon_F_hwreq_block_blockedhold_approach_sim.yaml` | `headon_v4_corr_trial1-10` | `headon_v4_corr_n10_report.txt` |
| Watch-first only | `headon_F_hwreq_block_blockedhold_sim.yaml` | `headon_final_corr_trial1-5` | `headon_final_corr_report.txt` |
| Sustained approach only | `…_approach_nowatch_sim.yaml` | `headon_v4nw_corr_trial1-5` | `headon_v4nw_corr_report.txt` |
| Final configuration | `…_approach_hwsync_rec20_noghost_groupskip_sim.yaml` | `headon_rec20gs_trial01-10` | `headon_rec20gs_n10_report.txt`, `headon_lane_start_report.txt` |

One trial:

```bash
CFG=config/social_nav2_headon_F_hwreq_block_sim.yaml ./run_headon_F_trial.sh headon_test_trial1
```

To place the walker elsewhere, use these variables:

| Variable | Effect |
|---|---|
| `PERSON_Y=-0.4` | walker offset 0.4 m to the robot's right |
| `WORLD=empty_human` | open space instead of the corridor |
| `PERSON_SPEED` | walking speed |
| `TRACK_DROPOUT_AT=<m>` | track-loss test |

Analysis of one or more head-on bags:

```bash
python3 analysis/analyse_headon_F.py bags/headon_test_trial*
```

### 4.3 Standing pair, wide and narrow (Chapter 6, Section 7.4)

| Thesis set | Configuration | Bags | Report in `analysis/` |
|---|---|---|---|
| Simulation version | `…_blockedhold_approach_sim.yaml` | `conv_F_v4_{wide,narrow}_trial1-10` | `conv_F_v4_n10_report.txt` |
| Before the fix / group skip | `…_blockedhold_sim.yaml` / `…_blockedhold_groupskip_sim.yaml` | `conv_F_final_*`, `conv_F_gs3_*` | `conv_F_final_wide_narrow_report.txt`, `conv_F_gs3_report.txt` |
| Final configuration | `…_rec20_noghost_groupskip_sim.yaml` | `conv_F_rec20gs_{wide,narrow}_trial01-10` | `conv_F_rec20gs_n10_report.txt`, `member_lanes_rec20_report.txt` |
| Stage 4 on the pair | rebuilt in `repro_ws` | `conv_F_stage4_narrow_trial1-6` | `conv_F_stage4_narrow_report.txt` |
| Controls (no groups, lethal zone) | run in `repro_ws` | `ctrl_*_trial1-6` | `conv_F_controls_report.txt` |

One trial and its analysis:

```bash
CASE=narrow CFG=config/social_nav2_headon_F_hwreq_block_blockedhold_approach_sim.yaml \
    ./run_conv_F_trial.sh conv_F_test_narrow_trial1
python3 analysis/analyse_F_narrow.py bags/conv_F_test_narrow_trial1
python3 analysis/aggregate_F_trials.py "conv_F_test_narrow_trial*"
```

### 4.4 The final configuration in one command (simulation side of Section 7.6)

```bash
./run_rec20ng_groupskip_test.sh              # headon, headon_y04, wide, narrow (10 trials each)
./run_rec20ng_groupskip_test.sh headon wide  # only some parts
python3 analysis/unmapped_clearance.py bags/headon_rec20gs_trial*   # clearance as compared with the robot
```

`run_rec20noghost_test.sh` runs the same configuration without group skip (bags `*_rec20ng_*`, report `unmapped_clearance_rec20ng_report.txt`).

### 4.5 Batch scripts of the tuning steps (8 and 9 October 2026)

Each script lists its bags and baseline in its header.

| Script | What it tested |
|---|---|
| `run_v4_followup.sh`, `run_v4_nowatch.sh` | sustained-approach critic (version 4) and its two rules separated |
| `run_hwsync_headon.sh`, `run_hwsync_split.sh`, `run_hwsync_pathside_batch.sh`, `run_pathside30.sh` (*host*) | the robot's changes ported to the simulation |
| `run_sd03_test.sh`, `run_rec_test.sh`, `run_rec15_test.sh`, `run_rec20_test.sh`, `run_rec20_y04_test.sh`, `run_rec35_test.sh` | side-decision time and side-switch offset |
| `run_conv_F_gskip_batch.sh` | group skip on the pair |
| `run_headon_F_batch.sh` | head-on avoidance attempts (early October) |

### 4.6 Older critic versions (`repro_ws`)

`repro_ws` holds the critic packages of commits `9d7b406` (baseline) and `b06d253` (stage 4), which were rebuilt on 10 October 2026. `record_problem1_stage4.sh` *(local)* replays the stage-4 narrow pair with Gazebo and RViz open, for a video. `repro_log_20261010.txt` *(local)* lists every reproduced trial.

The script of the pair controls (`run_conv_F_trial_control.sh`) is not on this laptop any more. The settings of those runs are in `repro_ws/bags/ctrl_*/run_notes.txt`.

---

## 5. If something goes wrong

| Symptom | Fix |
|---|---|
| `... not active after 120 s` | a start-up failure; run the trial again with `_a2` |
| no Gazebo, RViz or YOLO window | run `xhost +local:root` on the host |
| `bag already exists` | choose a new name; old bags are never overwritten |
| RViz crashes at start with CycloneDDS | the scripts use the default FastDDS on purpose; do not set `RMW_IMPLEMENTATION` |
| custom world or model not found | see "Custom Gazebo worlds/models" in `README.md`; the scripts sync them before launch |
