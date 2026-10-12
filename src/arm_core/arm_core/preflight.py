"""Reject unsupported configurations before starting any simulation process."""
import argparse
import json
import math
import os
from pathlib import Path
import sys

import yaml

REQUIRED_PACKAGES = ['ur_simulation_gz', 'ur_description', 'ur_moveit_config',
                     'ur_robot_driver', 'moveit_ros_move_group', 'moveit_ros_planning',
                     'ros_gz_sim', 'gz_ros2_control', 'rviz2', 'arm_core', 'arm_bringup']
FIELDS = {'config_version', 'sim_mode', 'robot_type', 'real_hardware', 'use_sim_time',
          'scenario_id', 'ros_domain_id', 'gz_partition', 'gazebo_gui', 'launch_rviz',
          'world_file', 'camera_required', 'startup_timeout_sec', 'stale_timeout_sec',
          'required_controllers', 'base_frame', 'tool_frame'}


def validate_config(config):
    if not isinstance(config, dict) or set(config) != FIELDS:
        raise ValueError('Configuration fields missing or unknown')
    version=config.get('config_version')
    if version not in ['s02.1','s05.1','s07.1']: raise ValueError('Unsupported configuration version')
    scenario={'s02.1':'bare_arm','s05.1':'planning_scene','s07.1':'tabletop'}[version]
    expected = {'config_version': version, 'sim_mode': 'gazebo', 'robot_type': 'ur5e',
                'real_hardware': False, 'use_sim_time': True, 'camera_required': version=='s07.1',
                'scenario_id': scenario, 'ros_domain_id': 91, 'gz_partition': 'ros2_s01_ur5e',
                'base_frame': 'base_link', 'tool_frame': 'grasp_tcp' if version=='s07.1' else 'tool0'}
    for key, value in expected.items():
        if type(config[key]) is not type(value) or config[key] != value:
            raise ValueError(f'Unsupported {key}: expected {value!r}')
    for key in ['gazebo_gui', 'launch_rviz']:
        if type(config[key]) is not bool:
            raise ValueError(key + ' must be a YAML boolean')
    for key in ['startup_timeout_sec', 'stale_timeout_sec']:
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + ' must be finite and positive')
    if config['required_controllers'] != ['joint_state_broadcaster', 'scaled_joint_trajectory_controller']+(['gripper_controller'] if version=='s07.1' else []):
        raise ValueError('Controller names must match the verified S01 baseline')
    if config['world_file'] != {'s02.1':'empty.sdf','s05.1':'generated:planning_scene','s07.1':'generated:tabletop'}[version]:
        raise ValueError('S02 baseline requires the official empty.sdf world')
    return config


def inspect_environment(config, environ=None, resolver=None):
    env = os.environ if environ is None else environ
    if env.get('ROS_DISTRO') != 'jazzy' or not env.get('AMENT_PREFIX_PATH'):
        raise ValueError('ROS environment is not sourced: source /opt/ros/jazzy/setup.bash and project overlays')
    if env.get('ROS_DOMAIN_ID') != str(config['ros_domain_id']) or env.get('GZ_PARTITION') != config['gz_partition']:
        raise ValueError('Wrong simulation domain/partition; use scripts/s02/start.sh')
    if resolver is None:
        from ament_index_python.packages import get_package_prefix
        resolver = get_package_prefix
    packages = {}
    for name in REQUIRED_PACKAGES:
        try:
            packages[name] = resolver(name)
        except LookupError as exc:
            raise ValueError('Missing package or overlay: ' + name) from exc
    fixed_prefix = str(Path.home()/'workspaces/moveit_shutdown_fix/install') + '/'
    for name in ['moveit_ros_move_group', 'moveit_ros_planning']:
        if not packages[name].startswith(fixed_prefix):
            raise ValueError('Required S01 shutdown fix overlay missing: ' + name)
    return packages


def load(path):
    return validate_config(yaml.safe_load(Path(path).read_text()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args = parser.parse_args(argv)
    try:
        config = load(args.config)
        packages = inspect_environment(config)
        print(json.dumps({'status': 'PASS', 'config_version': config['config_version'], 'packages': packages}))
        return 0
    except (ValueError, OSError, yaml.YAMLError) as exc:
        print('PREFLIGHT FAIL: ' + str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
