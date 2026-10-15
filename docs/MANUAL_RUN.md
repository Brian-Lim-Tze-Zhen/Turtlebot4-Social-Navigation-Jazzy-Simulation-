# Running a scenario by hand

`./run_scenario.sh <scenario>` does everything below in one command. This page
lists the same steps one at a time, so each part can be started, watched and
debugged on its own. The values are the ones the scripts use
(`ros2_ws/run_headon_F_trial.sh` for the two head-on scenarios,
`ros2_ws/run_conv_F_trial.sh` for the other three).

The robot's starting pose and the goal can be set by hand in RViz
(**2D Pose Estimate** and **Nav2 Goal**) or with a command; both are given.

| Scenario | Starting pose (on the dock) | Goal |
|---|---|---|
| headon, headon_blocked | (-3.0, 0.0), facing -x | (8.0, 0.0), facing +x |
| conversation_wide, conversation_narrow | (-1.0, 0.0), facing -x | (6.0, 0.0), facing +x |
| queue | (-1.0, 0.0), facing -x | (6.0, -3.0), facing +x |

The map frame is the Gazebo world frame, so these are also the positions in
Gazebo. The robot starts on its dock facing away from the people (-x);
undocking turns it round to face them (+x).

Every step runs **inside the container**. Each numbered terminal is a new shell:

```bash
docker exec -it thesis_social_nav bash
```

## 0. Pick a scenario

Paste the shell setup and one scenario block into **every** terminal you open.

Shell setup:

```bash
cd /root/thesis_social_navigation_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1
CL=src/social_perception/social_perception/camera_lidar
```

Scenario blocks (paste one):

```bash
# headon: one person walks toward the robot in a 2.5 m corridor
SCEN=headon WORLD=corridor_headon MAP=maps/corridor_headon_aligned.yaml \
CFG=config/social_nav2_headon_F_hwreq_block_blockedhold_sim.yaml SX=-3.0 GX=8.0 GY=0.0

# headon_blocked: two people stand across the corridor, then walk away
SCEN=headon_blocked WORLD=corridor_two_human MAP=maps/corridor_headon_aligned.yaml \
CFG=config/social_nav2_headon_F_hwreq_block_blockedhold_sim.yaml SX=-3.0 GX=8.0 GY=0.0

# conversation_wide: a standing pair in open space
SCEN=conversation_wide WORLD=conversation_test MAP=maps/map_name.yaml \
CFG=config/social_nav2_ablation_F_socialzone_sim.yaml SX=-1.0 GX=6.0 GY=0.0 QUEUE=false

# conversation_narrow: the same pair in a 2.0 m corridor
SCEN=conversation_narrow WORLD=conversation_test_narrow MAP=maps/conversation_test_narrow.yaml \
CFG=config/social_nav2_ablation_F_socialzone_sim.yaml SX=-1.0 GX=6.0 GY=0.0 QUEUE=false

# queue: four people in a line across the robot's way
SCEN=queue WORLD=queue_test MAP=maps/map_name.yaml \
CFG=config/social_nav2_ablation_F_socialzone_sim.yaml SX=-1.0 GX=6.0 GY=-3.0 QUEUE=true
```

## 1. Clean up and prepare (terminal 1)

Every run needs a fresh Gazebo. The DDS clean-up matters: stale FastDDS
shared-memory files left in `/dev/shm` by a previous run made Nav2's
start-up hang.

```bash
pkill -KILL -f "^gz sim"                  # a Gazebo left from an earlier run
timeout 10 ros2 daemon stop; pkill -KILL -f ros2cli.daemon
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_*

/usr/local/bin/setup_gazebo_worlds.sh     # copy the custom worlds and models

# The installed TurtleBot 4 launch files ignore `headless`; install the
# patched copies from the workspace (the originals are kept as *.orig).
D=/opt/ros/jazzy/share/turtlebot4_gz_bringup/launch
for f in sim.launch.py turtlebot4_gz.launch.py; do
  [ -f $D/$f.orig ] || cp $D/$f $D/$f.orig
  cp launch/$f $D/$f
done
```

## 2. Gazebo, TurtleBot 4 and Nav2 (terminal 1)

The robot spawns on its dock, facing away from the people.

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  world:=$WORLD slam:=false nav2:=true localization:=true rviz:=false \
  map:=$MAP params_file:=$CFG headless:=false \
  x:=$SX y:=0.0 yaw:=3.14159
```

RViz (terminal 2). It is needed for the hand-set pose and goal, and shows
the map, the costmaps, the people and the planned path:

```bash
rviz2 -d config/headon_view.rviz --ros-args -p use_sim_time:=true
```

To read map coordinates in RViz, pick **Publish Point** in the toolbar and
hover over the map: the status bar at the bottom shows the position under
the cursor.

## 3. Localise and undock (terminal 3)

Wait until Nav2 is up. This takes 20 to 60 s; if a node is not active after
about 2 minutes, stop terminal 1 and start again from step 1.

```bash
for n in /map_server /amcl /controller_server; do
  until ros2 lifecycle get $n 2>/dev/null | grep -q "^active"; do sleep 1; done
  echo "$n active"
done
sleep 5                                   # let AMCL's TF buffer catch up
```

Set the initial pose while the robot is still on its dock, in one of two
ways.

**By hand in RViz:** click **2D Pose Estimate** in the toolbar, press the
mouse on the robot's position (the starting pose in the table above), and
drag towards -x, away from the people and the goal, before releasing. The
arrow is the robot's heading.

**Or with a command** (terminal 3). In the message, the covariance comes
before the pose on purpose:

```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685], pose: {position: {x: $SX, y: 0.0, z: 0.0}, orientation: {z: 1.0, w: 0.0}}}}"
```

Check that AMCL took the pose. In RViz, the laser scan should now line up
with the walls of the map. In terminal 3:

```bash
timeout 4 ros2 run tf2_ros tf2_echo map odom | grep -m1 Translation
```

If no `Translation` line appears, AMCL rejected the pose (its TF buffer was
still behind); set the pose again.

Undock. The robot backs off the dock and turns to face the people:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

It should end with `SUCCEEDED`; if it does not, send it again.

## 4. Place the walkers (head-on scenarios only, terminal 3)

```bash
# headon: person_1 at the far end of the corridor
gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person_1", position: {x: 9.5, y: 0.0, z: 0.0}, orientation: {z: -0.70710678, w: 0.70710678}'

# headon_blocked: two people side by side in front of the robot
gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person_1", position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {z: -0.70710678, w: 0.70710678}'
gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "person_2", position: {x: 2.0, y: -0.5, z: 0.0}, orientation: {z: -0.70710678, w: 0.70710678}'
```

The person mover (step 7) talks to Gazebo through this bridge (terminal 4):

```bash
ros2 run ros_gz_bridge parameter_bridge "/world/$WORLD/set_pose@ros_gz_interfaces/srv/SetEntityPose"
```

## 5. Social zones and perception

Each command runs in its own terminal.

Social-zone layer and its costmap filter:

```bash
python3 $CL/social_zone_costmap_node_sim.py --ros-args -p use_sim_time:=true
```

```bash
ros2 launch launch/costmap_filter_info_sim.launch.py params_file:=$CFG
```

Perception: YOLO, camera-ray fusion, tracking and prediction. The
"YOLO ByteTrack Position" window opens once camera images arrive.

```bash
# headon, headon_blocked
ros2 launch launch/perception_camray_bringup_sim.launch.py \
  coast_timeout:=1.5 ray_coast:=true ray_coast_s:=1.5 \
  yolo_imgsz:=320 yolo_min_conf:=0.45 max_person_range:=8.0 \
  ellipse_a_slope:=0.0 ellipse_a_max:=0.3 pass_side_block_width:=0.0 \
  ellipse_b:=0.15 person_disk_radius:=0.30 kf_script:=human_kf_predictor_lidar.py

# conversation_wide, conversation_narrow, queue (launch defaults)
ros2 launch launch/perception_camray_bringup_sim.launch.py
```

Group detector:

```bash
# headon, headon_blocked
python3 $CL/social_group_detector_node_lidarhold_sim.py --ros-args \
  -p use_sim_time:=true -p costmap_topic:=/map -p show_debug_image:=false

# conversation_wide, conversation_narrow, queue
python3 $CL/social_group_detector_node_lidarhold_sim.py --ros-args \
  -p use_sim_time:=true -p costmap_topic:=/map -p show_debug_image:=false \
  -p queue_detection:=$QUEUE
```

Head-on scenarios only: the blocked-person beep and hold, and the goal retry.
Run each in its own terminal.

```bash
python3 $CL/blocked_person_beep_node_sim.py --ros-args -p use_sim_time:=true -p hold:=true
```

```bash
python3 $CL/beep_retry_node_sim.py --ros-args -p use_sim_time:=true
```

## 6. Check that perception is ready (terminal 3)

```bash
ros2 topic echo /predicted_person_cloud --once        # the pipeline publishes
ros2 node list | grep -E "yolo|group"                 # the nodes are alive
```

For the three standing-group scenarios, wait until the group is reported. The
second field should be `conversation`, or `queue` for the queue scenario:

```bash
ros2 topic echo /social_groups --once
```

"Ready" here only means the topics have publishers. If the YOLO window does
not appear, check the terminal of step 5 for a Python error.

## 7. Record, send the goal and start the walkers

Optional: record a bag for the analysis (terminal 5). The head-on analysis
reads `bags/<name>/data`, the others read `bags/<name>` together with a copy
of the world file.

```bash
NAME=manual_${SCEN}_$(date +%H%M%S)
case $SCEN in headon*) OUT=bags/$NAME/data ;; *) OUT=bags/$NAME ;; esac
ros2 bag record -o $OUT --qos-profile-overrides-path analysis/tf_qos_override.yaml \
  --topics /clock /person_ground_truth /person2_ground_truth /sim_ground_truth_pose /odom /amcl_pose \
           /tf /tf_static /plan /scan /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /collision_monitor_state \
           /optimal_trajectory /person_positions_map /camera_ray_clusters /person_positions_fused \
           /predicted_person_positions /predicted_person_cloud /social_groups /social_zone_map \
           /social_critic/lane_markers /social_critic/lane_block /rosout /cmd_audio \
           /blocked_person_event /blocked_hold_polygon
```

In terminal 3, once the recorder is running:

```bash
cp /opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds/$WORLD.sdf bags/$NAME/world_used.sdf
```

Send the goal, in one of two ways.

**By hand in RViz:** click **Nav2 Goal** in the toolbar, press the mouse on
the goal position (the table at the top), and drag towards +x, the direction
of travel, before releasing. The Navigation 2 panel shows the
goal's progress, and the planned path appears on the map.

**Or with a command** (terminal 3). It prints `Goal accepted` and, at the
end, the final status. The third argument is the timeout in wall seconds;
the fourth is a file it creates when the goal is accepted.

```bash
python3 headon_send_goal.py $GX $GY 300 /tmp/goal_accepted
```

Head-on scenarios: start the walkers as soon as the goal is accepted, that
is, when the robot starts to move or `Goal accepted` appears (terminal 6).
The scripts start the person at that moment.

```bash
# headon: person_1 walks toward the robot at 1.2 m/s
python3 src/social_perception/social_perception/move_person_oneway.py --ros-args -p use_sim_time:=true \
  -p world_name:=$WORLD -p model_name:=person_1 \
  -p point_a:="[9.5, 0.0]" -p point_b:="[-2.0, 0.0]" -p speed:=1.2
```

```bash
# headon_blocked: wait about 20 s (the robot stops and beeps), then the pair walks away together
M=src/social_perception/social_perception/move_person_oneway.py
python3 $M --ros-args -p use_sim_time:=true -p world_name:=$WORLD -p model_name:=person_1 \
  -p point_a:="[2.0, 0.5]" -p point_b:="[9.5, 0.5]" -p speed:=1.2 &
python3 $M --ros-args -p use_sim_time:=true -p world_name:=$WORLD -p model_name:=person_2 \
  -p point_a:="[2.0, -0.5]" -p point_b:="[9.5, -0.5]" -p speed:=1.2 \
  -p ground_truth_topic:=/person2_ground_truth
```

`run_scenario.sh` holds the pair for exactly 20 s of simulation time
(`STAND_S`) and starts both walkers at the same moment; by hand, the timing
is only approximate.

## 8. Stop and analyse

Stop the recorder with Ctrl+C first, so the bag is closed properly, then
stop every other terminal with Ctrl+C and run `pkill -KILL -f "^gz sim"`.

```bash
python3 analysis/analyse_headon_F.py bags/$NAME                    # headon, headon_blocked
python3 analysis/blocked_beep_timeline.py bags/$NAME/data          # headon_blocked: beeps
python3 analysis/analyse_F_narrow.py bags/$NAME                    # conversation_wide / _narrow
python3 analysis/queue_run_summary.py bags/$NAME $GX $GY           # queue
```

The one-command runs also save the configuration, the live Nav2 parameters
and checksums of every node next to the bag; hand runs do not.
