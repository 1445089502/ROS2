# ROS 2 UR5e Simulation: Manipulation Stack

A simulation-only ROS 2 project integrating a Universal Robots UR5e arm with Gazebo Harmonic and MoveIt 2. The S07 baseline adds a Robotiq 2F-85 gripper, a fixed RGB-D camera, a tabletop scene, and a bounded task interface for safe simulation workflows.

## What is included

- ROS 2 packages for bringup, task management, interfaces, MoveIt scene setup, simulation, vision, voice, and agent integration.
- Gazebo simulation with an UR5e, gripper, RGB-D camera, and tabletop objects.
- MoveIt planning-scene checks and task-level validation, cancellation, idempotency, and stop handling.
- Source tests and configuration for the verified S07 simulation baseline.

The repository is simulation-only. It has not been connected to a physical robot. The camera is integrated, but autonomous visual object detection and grasping are not part of S07. The later S07b known-position grasp work is intentionally not included in this release snapshot.

## Environment

The S07 baseline was exercised on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and MoveIt 2. The recorded local acceptance covered 90 runtime checks and three scene resets. Raw logs, sensor captures, and machine-specific evidence are not included here. A clean-machine reproduction has not yet been verified.

## Build and run

Install ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, the Universal Robots simulation packages, and the Robotiq description package listed in [`config/s07_dependencies.lock.yaml`](config/s07_dependencies.lock.yaml). In a ROS-enabled shell at the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
```

Launch the tabletop simulation and MoveIt:

```bash
ARM_TASK_MANAGER=1 ros2 launch arm_bringup sim_bringup.launch.py \
  config:=$(ros2 pkg prefix arm_simulation --share)/config/tabletop.yaml
```

The exact package availability and GUI behavior depend on the ROS/Gazebo installation. See [S07 details](docs/s07/README.md) for the robot model, topics, and task interface.

