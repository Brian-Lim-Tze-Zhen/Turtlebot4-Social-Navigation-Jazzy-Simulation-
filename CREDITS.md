# Credits and third-party software

This repository builds on the work of others. Numbers in brackets are the references of the thesis *Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding*.

## Included in this repository

| What | Where | Origin | License |
|---|---|---|---|
| `person_standing` Gazebo model | `ros2_ws/simulation_models/person_standing/` | Marina Kollmitz (University of Freiburg), made with MakeHuman; author kept in `model.config` | see `model.config` |
| Patched TurtleBot 4 simulation launch files (`headless` support) | `ros2_ws/launch/sim.launch.py`, `turtlebot4_gz.launch.py`, and their copies in `ablation_ws/` | `turtlebot4_gz_bringup`, Clearpath Robotics; copyright headers kept | Apache-2.0 |
| Behaviour trees | `ros2_ws/behavior_trees/`, `ablation_ws/behavior_trees/` | modified from the navigate-to-pose tree of Nav2 | Apache-2.0 |
| Nav2 parameter files | `ros2_ws/config/`, `ablation_ws/config/` | start from the Nav2 and TurtleBot 4 parameter files, with the thesis changes | Apache-2.0 |

## Used, not included

Installed by the Dockerfile or cloned during set-up.

| Software | Author | License | Used for |
|---|---|---|---|
| [ROS 2](https://docs.ros.org/) [20] | Open Robotics and contributors | Apache-2.0 | middleware |
| [Nav2](https://github.com/ros-navigation/navigation2) [5, 21] | Steve Macenski and contributors | Apache-2.0, BSD-3-Clause, LGPL-2.1 (AMCL) | navigation stack: costmaps, KeepoutFilter, NavFn, behaviour tree, collision monitor |
| [Nav2 MPPI controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) [23, 24] | Steve Macenski, Aleksei Budyakov | MIT | the controller the SocialCritic plugs into (the critic builds on its `CriticFunction` interface) |
| [AMCL](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl) [27] | Nav2 contributors | LGPL-2.1-or-later | localisation |
| [nonpersistent_voxel_layer](https://github.com/SteveMacenski/nonpersistent_voxel_layer) [5, 21] | Steve Macenski | BSD | costmap layer that takes the person cloud |
| [TurtleBot 4 packages](https://github.com/turtlebot/turtlebot4) [70] | Clearpath Robotics | Apache-2.0 | robot description, bring-up and navigation launch files |
| [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) [38] | Glenn Jocher, Ayush Chaurasia, Jing Qiu | AGPL-3.0 | person and pose detection |
| [ByteTrack](https://github.com/ifzhang/ByteTrack) [46] | Yifu Zhang et al. | MIT | multi-object tracking (as built into Ultralytics) |
| [OpenVINO](https://github.com/openvinotoolkit/openvino) [78] | Intel | Apache-2.0 | YOLO inference on the CPU |
| [Gazebo Harmonic](https://gazebosim.org/) [66] | Open Robotics | Apache-2.0 | simulator |
| [Warehouse, person_standing (Gazebo Fuel)](https://app.gazebosim.org/OpenRobotics) | Open Robotics | see each model | downloaded at build |
