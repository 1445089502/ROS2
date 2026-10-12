"""S07 compatibility gate: official UR launch plus installed Robotiq geometry."""
import json
from pathlib import Path
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from arm_moveit.scene import load,generate_world


def setup(context):
    folder=Path(LaunchConfiguration('session_dir').perform(context));folder.mkdir(parents=True,exist_ok=True)
    if (folder/'session.json').exists():raise RuntimeError('Session already exists')
    ur=Path(share('ur_simulation_gz'));package=Path(share('arm_simulation'))
    config=yaml.safe_load((ur/'config/ur_controllers.yaml').read_text())
    config['controller_manager']['ros__parameters']['gripper_controller']={'type':'joint_trajectory_controller/JointTrajectoryController'}
    config['gripper_controller']={'ros__parameters':{'joints':['robotiq_85_left_knuckle_joint'],
      'command_interfaces':['position'],'state_interfaces':['position','velocity'],
      'constraints':{'goal_time':2.,'stopped_velocity_tolerance':.02,'robotiq_85_left_knuckle_joint':{'trajectory':.1,'goal':.01}}}}
    controllers=folder/'controllers.yaml';controllers.write_text(yaml.safe_dump(config))
    scene=Path(share('arm_moveit'))/'config/scene.yaml';world=folder/'world.sdf'
    generate_world(load(scene),'/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf',world)
    extra=[]
    if LaunchConfiguration('camera').perform(context)=='true':
        from arm_simulation.tabletop import extend_world,bridge_config,camera_pose
        data=yaml.safe_load((package/'config/tabletop.yaml').read_text());(folder/'tabletop.yaml').write_text(yaml.safe_dump(data))
        extend_world(world,data);bridge_config(data,folder/'bridge.yaml')
        xyzrpy=camera_pose(data);cam=data['camera']
        extra=[Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(folder/'bridge.yaml')}],output='screen'),
          Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--x',str(xyzrpy[0]),'--y',str(xyzrpy[1]),'--z',str(xyzrpy[2]),'--roll',str(xyzrpy[3]),'--pitch',str(xyzrpy[4]),'--yaw',str(xyzrpy[5]),'--frame-id','world','--child-frame-id',cam['frame']],output='screen'),
          Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--roll','-1.57079632679','--pitch','0','--yaw','-1.57079632679','--frame-id',cam['frame'],'--child-frame-id',cam['optical_frame']],output='screen')]
    (folder/'session.json').write_text(json.dumps({'session_id':str(uuid.uuid4()),'stage':'S07','scope':'GRIPPER_AND_RGBD' if extra else 'GRIPPER_COMPATIBILITY_ONLY','real_hardware':False}))
    simulation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ur/'launch/ur_sim_control.launch.py')),
      launch_arguments={'ur_type':'ur5e','launch_rviz':'false','world_file':str(world),
       'description_file':str(package/'urdf/ur5e_gripper.urdf.xacro'),'controllers_file':str(controllers)}.items())
    spawner=Node(package='controller_manager',executable='spawner',arguments=['gripper_controller','-c','/controller_manager','--controller-manager-timeout','60'],output='screen')
    return [SetLaunchConfiguration('on_exit_shutdown','true'),SetLaunchConfiguration('sigterm_timeout','15'),simulation,spawner]+extra


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session_dir'),DeclareLaunchArgument('camera',default_value='false'),OpaqueFunction(function=setup)])
