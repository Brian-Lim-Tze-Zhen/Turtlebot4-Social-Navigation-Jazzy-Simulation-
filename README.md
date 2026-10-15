# Social Navigation for TurtleBot 4 (ROS 2 Jazzy, simulation)

Simulation code of the master's thesis *Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding* (Deggendorf Institute of Technology, Campus Cham, 2026).

A TurtleBot 4 detects people with its camera and LiDAR (YOLO and ByteTrack), predicts their motion with a Kalman filter, and passes them at a social distance. Nav2's MPPI controller uses a custom critic (SocialCritic) with lane rules for a person walking towards the robot, and a group detector that tells the robot whether to go around a standing pair or pass between its members.

The same system on the physical TurtleBot 4 (ROS 2 Humble) is in the companion repository [Turtlebot4-Hardware-Humble](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble).

## How it works

![The final configuration from the camera to the wheels](docs/media/final_configuration.png)

*The final configuration from the camera to the wheels (thesis Figure 5.2, Section 5.3). Blue: existing components (sensors, detector, tracker, Nav2 and the base); orange: nodes developed in the thesis; green: the SocialCritic with the lane rules. The labelled arrows into the costmaps carry what these nodes write into them. The simulation runs the same configuration with `_sim` copies of the nodes, which read raw camera images and use no `/turtlebot4` namespace. The ROS topics between the nodes, and the simulation and robot files side by side, are in [docs/CONFIGURATIONS.md](docs/CONFIGURATIONS.md).*

## Results

### Head-on walker: the robot moves aside early

![Head-on walker in simulation](docs/media/headon_sim.gif)

*Simulation, 2.5 m corridor, walker at 1.2 m/s, played at 2x speed. This is a demonstration run of 4 October 2026 with the lane rules; the robot passed on the right at 0.74 m between centres without stopping.*

Ten trials per configuration (`ros2_ws/analysis/*_n10_report.txt`). The target is 0.8 m between the centres of robot and person.

| Configuration | Closest distance between centres (m) | Stopped (s) | Time to goal (s) | Goal reached |
|---|---|---|---|---|
| Baseline: person cloud, no lane rules | 0.08 ± 0.07 (contact) | 2.4 ± 2.5 | 42.8 ± 1.6 | 10 / 10 |
| Lane rules | 0.84 ± 0.04 | 0.0 | 37.3 ± 0.4 | 10 / 10 |
| Final configuration (as on the robot) | 0.76 ± 0.07 | 0.0 | 37.7 ± 0.4 | 10 / 10 |

### Standing pair: around in open space, between in a narrow corridor

![Standing pair in simulation](docs/media/conversation_pair_sim.gif)

*Simulation, played at 2x speed. Left: in open space the robot goes around the pair. Right: in a 2.0 m corridor there is no way around, so it passes between the two people without stopping. Demonstration runs of 4 October 2026 (`demo_conversation_wide_20261004_132727`, `demo_conversation_narrow_20261004_132106`), not trials of the tables.*

Final configuration in simulation, ten trials each (`ros2_ws/analysis/conv_F_rec20gs_n10_report.txt`):

| Case | Robot's path | Closest distance between centres (m) | Stopped (s) | Time to goal (s) | Goal reached |
|---|---|---|---|---|---|
| Wide (open space) | around the pair | 0.87 ± 0.03 | 0.0 | 24.6 ± 0.2 | 10 / 10 |
| Narrow (2.0 m corridor) | between the members | 0.71 ± 0.02 | 0.0 | 21.4 ± 0.2 | 10 / 10 |

How to repeat these runs: [HOW_TO_RUN.md](HOW_TO_RUN.md), section 4.

### On the real robot

The final configuration was also run on the physical TurtleBot 4, ten runs per case (thesis Section 7.6.2): the robot passed the head-on walker on the right in all ten runs, went around the pair in the wide hallway and between the members in the narrow section in all ten, without stopping for the pair. Distances, run commands and run records are in the [hardware repository](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble).

## Quick start

You need Linux with Docker and Docker Compose, an X11 display, and Intel/Mesa graphics. For an NVIDIA GPU, use `docker compose -f docker-compose.nvidia.yaml` in place of `docker compose` (needs the NVIDIA Container Toolkit).

```bash
git clone https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Social-Navigation-Jazzy-Simulation-.git
cd Turtlebot4-Social-Navigation-Jazzy-Simulation-/docker
docker compose build && docker compose up -d
xhost +local:root

docker exec -it thesis_social_nav bash
cd /root/thesis_social_navigation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
./run_scenario.sh headon        # or headon_blocked, conversation_wide, conversation_narrow, queue
```

To run a scenario step by step instead, one node per terminal with the pose and goal set by hand in RViz, follow [docs/MANUAL_RUN.md](docs/MANUAL_RUN.md). (`./run_sim.sh` starts the first pipeline of the project, not the one behind the results; see `ros2_ws/src/social_perception/social_perception/README.md`.)

The image installs ROS 2 Jazzy, Nav2, Gazebo Harmonic and the TurtleBot 4 packages; the YOLO weights are downloaded on first use.

## Documentation

In the order a new reader needs them:

| File | What it holds |
|---|---|
| [SCENARIOS.md](SCENARIOS.md) | the five demo scenarios, what to look for, and what to do when a run fails |
| [docs/MANUAL_RUN.md](docs/MANUAL_RUN.md) | the same scenarios step by step, one node per terminal, with the pose and goal set by hand in RViz (for debugging) |
| [docs/CONFIGURATIONS.md](docs/CONFIGURATIONS.md) | what the parts of the long configuration names mean, which configuration each run uses, and what differs between the simulation and the robot |
| [HOW_TO_RUN.md](HOW_TO_RUN.md) | folder map, set-up, and the command, recordings and report of every simulation experiment of the thesis |
| [docs/DEVELOPMENT_NOTES.md](docs/DEVELOPMENT_NOTES.md) | how the scenarios were built and tuned, with the runs of 24 September to 4 October 2026 |
| [Turtlebot4-Hardware-Humble](https://github.com/Brian-Lim-Tze-Zhen/Turtlebot4-Hardware-Humble) | the robot side: final robot configuration, launch and run scripts, results on the robot |
| [docker/notes/](docker/notes/) | fixes for problems of the container (TF QoS, YOLO threads, world sync, package pins) |

## Custom Gazebo worlds/models - known issue and workaround

`turtlebot4_gz_bringup`'s launch file does not reliably resolve
`model://` URIs via `GZ_SIM_RESOURCE_PATH` for nested custom models when
Gazebo is spawned through `ros2 launch` (confirmed empirically: works for
a standalone `gz sim` call, fails under `ros2 launch` with the identical
environment). `docker/setup_gazebo_worlds.sh` works around this by
copying custom worlds/models into `turtlebot4_gz_bringup`'s own installed
`worlds/` directory at container startup, and is also called explicitly
at the top of `run_sim.sh` for non-interactive invocations.

## Credits and third-party software

### Included in this repository

| What | Where | Origin | License |
|---|---|---|---|
| `person_standing` Gazebo model | `ros2_ws/simulation_models/person_standing/` | Marina Kollmitz (University of Freiburg), made with MakeHuman; author kept in `model.config` | see `model.config` |
| Patched TurtleBot 4 simulation launch files (`headless` support) | `ros2_ws/launch/sim.launch.py`, `turtlebot4_gz.launch.py`, and their copies in `ablation_ws/` | `turtlebot4_gz_bringup`, Clearpath Robotics; copyright headers kept | Apache-2.0 |
| Behaviour trees | `ros2_ws/behavior_trees/`, `ablation_ws/behavior_trees/` | modified from the navigate-to-pose tree of Nav2 | Apache-2.0 |
| Nav2 parameter files | `ros2_ws/config/`, `ablation_ws/config/` | start from the Nav2 and TurtleBot 4 parameter files, with the thesis changes | Apache-2.0 |

### Used, not included

Installed by the Dockerfile or cloned during set-up. Numbers in brackets are the references of the thesis.

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
