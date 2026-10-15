# social_perception

Perception, prediction, group and zone nodes, and the scripts that move the
simulated people. The folder holds the current pipeline and the earlier ones
it grew from; the earlier ones are kept because older bags, `run_sim.sh` and
some launch files still refer to them.

## Used by the thesis results and the demos

Everything the trial scripts (`run_headon_F_trial.sh`, `run_conv_F_trial.sh`,
`run_conv_F_trial_hwsync.sh`) and `launch/perception_camray_bringup_sim.launch.py`
start is in `camera_lidar/`, plus the walker script. The nodes are started with
`python3 <file>`, not `ros2 run`.

| File | Role |
|---|---|
| `camera_lidar/yolo_leg_detector_lidar_sim.py` | YOLO pose and ByteTrack on the camera image, publishes `/person_positions_map` |
| `camera_lidar/camera_ray_person_node_sim.py` | turns each detection into a ray from the camera and matches it with LiDAR |
| `camera_lidar/camera_ray_identity_node_sim.py` | keeps track identities across camera and LiDAR |
| `camera_lidar/human_kf_predictor_lidar.py` | Kalman-filter prediction (demos and earlier stages) |
| `camera_lidar/human_kf_predictor_lidar_noghost.py` | the same, dropping a coasting track that runs into the robot (`*hwsync*` configurations, final) |
| `camera_lidar/predicted_person_cloud_node_lidar.py` | the predicted people as a point cloud for the costmaps |
| `camera_lidar/person_marker_publisher.py` | markers for RViz |
| `camera_lidar/social_group_detector_node_lidarhold_sim.py` | finds standing pairs and queues, publishes `/social_groups` |
| `camera_lidar/social_zone_costmap_node_sim.py` | the graded social zone around a group, as a KeepoutFilter mask |
| `camera_lidar/blocked_person_beep_node_sim.py`, `_front_sim.py`, `_front_atgoal_sim.py` | hold the robot and beep while a person blocks it (the `_front_atgoal` one is final) |
| `camera_lidar/beep_retry_node_sim.py`, `beep_retry_node_hwsync_sim.py` | send the goal again after Nav2 gives up while blocked |
| `camera_lidar/person_stop_beep_node_sim.py` | stop-and-beep for the `*stopbeep*` configuration |
| `camera_lidar/track_dropout_relay_sim.py` | test tool: makes the SocialCritic lose a track (`TRACK_DROPOUT_AT`) |
| `move_person_oneway.py` | walks a simulated person from one point to another |

The robot's copies of these files, and the differences, are listed in
`docs/CONFIGURATIONS.md`.

## Earlier pipelines (not used by the current runs)

| Files | What they were |
|---|---|
| `camera_lidar/social_group_detector_node_sim.py` | group detector before the LiDAR hold; imports `open_clip`, which the image no longer installs |
| `yolo_detector.py`, `human_kf_predictor.py`, `predicted_person_cloud_node.py`, `prediction_marker_node.py`, `corridor_yield_node.py`, `leg_detector_node.py`, `identity_fusion_node.py`, `lidar_person_fusion.py` | the first pipeline (YOLO with depth, before camera rays). `run_sim.sh` and `launch/social_perception.launch.py`, `launch/headon_scenario.py` still start it |
| `group_formation_detector.py`, `social_group_cloud_node.py`, `queue_ground_truth_node.py` | the first group and queue detection (`launch/queue_perception.launch.py`). Imports `open_clip` (MobileCLIP), which the image no longer installs |
| `Headon/`, `Conversation/`, `Queue/`, `Lidar/` | per-scenario copies of the first pipeline |
| `move_person_gazebo.py`, `move_person_gazebo2.py`, `move_person_crossing*.py`, `move_person_combined.py`, `move_person_multi.py` | earlier walker scripts |
| `kf_prediction_logger.py`, `min_distance_logger.py` | early logging tools |
