# S07: Gripper, RGB-D Camera, and Tabletop Simulation

S07 is the verified simulation baseline for an UR5e arm with a Robotiq 2F-85 gripper, a fixed RGB-D camera, and three tabletop cubes. The local validation recorded 90 passing runtime checks and three successful scene resets. The raw run evidence is not part of this public snapshot.

## Run

Use Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, the Universal Robots simulation packages, and the Robotiq description package. From the repository root in a ROS-enabled shell:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
ARM_TASK_MANAGER=1 ros2 launch arm_bringup sim_bringup.launch.py \
  config:=$(ros2 pkg prefix arm_simulation --share)/config/tabletop.yaml
```

Dependency version details are recorded in [`config/s07_dependencies.lock.yaml`](../../config/s07_dependencies.lock.yaml). The launch starts Gazebo and MoveIt; verify installed package names and GUI support for the chosen platform.

## Key interfaces

- Gripper action: `/gripper_controller/follow_joint_trajectory`.
- Joint state: `/joint_states`.
- RGB image: `/camera/color/image_raw` (`rgb8`).
- Depth image: `/camera/depth/image_raw` (`32FC1`, metres).
- Each camera stream has a matching `camera_info` topic.

`tool0` is the arm mounting frame. `grasp_tcp` is the modeled center of the gripper opening. S07 integrates the camera stream but does not implement autonomous object detection or visual grasping. Closing the gripper alone is not treated as proof of a successful grasp.

## Source map

- `src/arm_simulation/`: URDF/Xacro model, tabletop world, camera, and Gazebo setup.
- `src/arm_moveit/`: planning scene and collision guards.
- `src/arm_core/`: health checks, task policy, task manager, and robot adapter.
- `src/arm_interfaces/`: ROS 2 service and action definitions.
- `src/arm_bringup/`: simulation launch and configuration.
