# analysis

Analysis scripts and the reports of every simulation experiment of the
thesis. Most of the 250 or so files here are reports (`*_report.txt`,
`*_rerun.txt`), one per set of trials; `HOW_TO_RUN.md`, section 4, says which
report belongs to which thesis table. The scripts below are the ones a reader
needs; the others are one-off analyses named after the experiment they belong
to.

Run them inside the container, from `/root/thesis_social_navigation_ws`:

| Script | For | Example |
|---|---|---|
| `analyse_headon_F.py` | head-on trials: goal, time, closest approach, stops; mean ± SD over several bags | `python3 analysis/analyse_headon_F.py bags/headon_rec20gs_trial*` |
| `blocked_beep_timeline.py` | `headon_blocked`: robot motion, beeps and block events over time | `python3 analysis/blocked_beep_timeline.py bags/<bag>/data` |
| `analyse_F_narrow.py` | one standing-pair trial (wide or narrow): path, clearance, group detection, zone | `python3 analysis/analyse_F_narrow.py bags/<bag>` |
| `aggregate_F_trials.py` | mean ± SD over pair trials already run through `analyse_F_narrow.py` | `python3 analysis/aggregate_F_trials.py "conv_F_test_narrow_trial*"` |
| `queue_run_summary.py` | one queue trial: distance to each member, where the robot crossed the line | `python3 analysis/queue_run_summary.py bags/<bag> 6.0 -3.0` |
| `unmapped_clearance.py` | the clearance measured as on the robot (nearest LiDAR return not on the map), for comparison with the robot runs | `python3 analysis/unmapped_clearance.py bags/headon_rec20gs_trial*` |
| `run_ablation_rerun_analyses.sh` | all analyses of the A to E ablation (Chapter 4, Section 7.2); runs on the host | `ros2_ws/analysis/run_ablation_rerun_analyses.sh` |

`tf_qos_override.yaml` is not an analysis script: the trial scripts pass it to
`ros2 bag record` so that `/tf` is recorded reliably; without it about half
of the bags had no `map -> odom` transform (`docker/notes/tf-qos-dropout.md`).

Robot and person touch at 0.44 m centre to centre (robot radius 0.189 m plus
body radius 0.25 m); the head-on and pair reports give both the centre and
the surface distance.
