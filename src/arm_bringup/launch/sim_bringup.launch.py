"""Official UR simulation -> observed controller/TF readiness -> MoveIt -> RViz."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from arm_core.preflight import load, inspect_environment


def setup(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).resolve()
    config = load(config_path)
    packages = inspect_environment(config)
    folder = Path(LaunchConfiguration('session_dir').perform(context)).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/'session.json').exists():
        raise RuntimeError('Refuse to overwrite an existing session: '+str(folder))
    session = {'session_id': str(uuid.uuid4()), 'task_id': 'bootstrap',
               'scenario_id': config['scenario_id'], 'config_version': config['config_version'],
               'started_at': datetime.now().astimezone().isoformat(), 'config': config,
               'config_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(), 'packages': packages,
               'mode': 'GAZEBO_SIMULATION', 'real_hardware': False}
    world_file=config['world_file']
    tabletop_enabled=config['scenario_id']=='tabletop'
    scene_enabled=config['scenario_id'] in ['planning_scene','tabletop']
    bundle=None
    if tabletop_enabled:
        from arm_simulation.tabletop import prepare
        bundle=prepare(folder);world_file=bundle['world'];session['world_sha256']=hashlib.sha256(Path(world_file).read_bytes()).hexdigest()
    elif scene_enabled:
        from arm_moveit.scene import load as load_scene,generate_world
        scene_path=Path(get_package_share_directory('arm_moveit'))/'config/scene.yaml'
        (folder/'scene.yaml').write_bytes(scene_path.read_bytes())
        official_world=Path(get_package_share_directory('gz_sim_vendor'))/'gz/gz-sim8/worlds/empty.sdf'
        if not official_world.exists(): official_world=Path('/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf')
        world_file=str(folder/'world.sdf')
        session['world_sha256']=generate_world(load_scene(scene_path),official_world,world_file)
    temp=folder/'session.tmp';temp.write_text(json.dumps(session,indent=2));temp.replace(folder/'session.json')
    (folder/'sim.yaml').write_bytes(config_path.read_bytes())
    official_sim = Path(get_package_share_directory('ur_simulation_gz'))/'launch/ur_sim_control.launch.py'
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(official_sim)), launch_arguments={
        'ur_type': config['robot_type'], 'launch_rviz': 'false',
        'gazebo_gui': str(config['gazebo_gui']).lower(), 'world_file': world_file,
        **({'description_file':bundle['description_file'],'controllers_file':bundle['controllers_file']} if bundle else {})}.items())
    # Use official MoveIt settings; align SRDF's robot name with the official URDF name "ur".
    moveit = (MoveItConfigsBuilder('ur', package_name='ur_moveit_config')
              .robot_description_semantic(Path('srdf')/'ur.srdf.xacro', {'name': 'ur'})
              .to_moveit_configs())
    if bundle:
        from arm_simulation.tabletop import extend_moveit
        extend_moveit(moveit,bundle['description'])
        (folder/'robot.srdf').write_text(moveit.robot_description_semantic['robot_description_semantic'])
    group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=[moveit.to_dict(), {'use_sim_time': True, 'publish_robot_description_semantic': True}])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2_moveit', output='screen',
                arguments=['-d', str(Path(get_package_share_directory('ur_moveit_config'))/'config/moveit.rviz')],
                parameters=[moveit.robot_description, moveit.robot_description_semantic,
                            moveit.robot_description_kinematics, moveit.planning_pipelines,
                            moveit.joint_limits, {'use_sim_time': True}])

    def health(phase):
        return Node(package='arm_core', executable='health_node', output='screen',
                    arguments=['--config', str(config_path), '--session-dir', str(folder), '--phase', phase])

    base_gate, moveit_gate, monitor = health('base'), health('moveit'), health('monitor')
    motion_path=Path(get_package_share_directory('arm_bringup'))/'config/motion.yaml'
    if tabletop_enabled:
        import yaml
        motion=yaml.safe_load(motion_path.read_text());motion.update(tool_frame='grasp_tcp',config_version='s07.1')
        motion_path=folder/'motion_input.yaml';motion_path.write_text(yaml.safe_dump(motion))
    task_manager = Node(package='arm_core', executable='task_manager', output='screen',
                        arguments=['--session-dir', str(folder), '--config',
                        str(motion_path)])
    task_enabled = os.environ.get('ARM_TASK_MANAGER', '0') == '1'
    scene_gate=Node(package='arm_moveit',executable='scene_setup',output='screen',arguments=['--session-dir',str(folder)]) if scene_enabled else None

    def advance(next_actions):
        def handle(event, ctx):
            if ctx.is_shutdown:
                return []
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Readiness check failed; inspect health.jsonl'))]
            return next_actions
        return handle

    def critical_exit(event, ctx):
        return [] if ctx.is_shutdown else [EmitEvent(event=Shutdown(reason='Required process exited'))]

    ready_actions=([rviz] if config['launch_rviz'] else [])+[monitor]+([task_manager] if task_enabled else [])
    return [
        SetLaunchConfiguration('on_exit_shutdown', 'true'),
        # WSL GUI teardown can exceed launch's default 5 s; keep a bounded grace period.
        SetLaunchConfiguration('sigterm_timeout', '15' if scene_enabled else '5'),
        RegisterEventHandler(OnProcessExit(target_action=base_gate, on_exit=advance([group, moveit_gate]))),
        RegisterEventHandler(OnProcessExit(target_action=moveit_gate, on_exit=advance(
            [scene_gate] if scene_enabled else ready_actions))),
        *([RegisterEventHandler(OnProcessExit(target_action=scene_gate,on_exit=advance(ready_actions)))] if scene_enabled else []),
        RegisterEventHandler(OnProcessExit(target_action=group, on_exit=critical_exit)),
        RegisterEventHandler(OnProcessExit(target_action=monitor, on_exit=critical_exit)),
        *([RegisterEventHandler(OnProcessExit(target_action=task_manager, on_exit=critical_exit))] if task_enabled else []),
        *([RegisterEventHandler(OnProcessExit(target_action=bundle['initializer'],on_exit=advance([])))] if bundle else []),
        simulation, base_gate,*(bundle['nodes'] if bundle else []),
    ]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('arm_bringup'))/'config/sim.yaml')
    default_session = os.environ.get('PROJECT_SESSION_DIR', str(Path.home()/'.ros/arm_sessions'/str(uuid.uuid4())))
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default_config),
                              DeclareLaunchArgument('session_dir', default_value=default_session),
                              OpaqueFunction(function=setup)])
