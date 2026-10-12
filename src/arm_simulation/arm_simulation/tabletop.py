"""Extend the verified world with physical cubes and one fixed aligned RGB-D sensor."""
import math
from pathlib import Path
from xml.etree import ElementTree as ET
import yaml


def add(parent,tag,value=None,**attrs):
    node=ET.SubElement(parent,tag,attrs)
    if value is not None:node.text=str(value)
    return node


def camera_pose(config):
    camera=config['camera'];xyz=camera['position'];d=[b-a for a,b in zip(xyz,camera['look_at'])]
    return xyz+[0.,math.atan2(-d[2],math.hypot(d[0],d[1])),math.atan2(d[1],d[0])]


def extend_world(path,config):
    tree=ET.parse(path);world=tree.getroot().find('world')
    size=config['cube_size'];mass=config['cube_mass']
    for cube in config['cubes']:
        model=add(world,'model',name=cube['id']);add(model,'pose',' '.join(map(str,cube['position']+[0,0,0])))
        link=add(model,'link',name='body');inertial=add(link,'inertial');add(inertial,'mass',mass);inertia=add(inertial,'inertia')
        for axis in ['ixx','iyy','izz']:add(inertia,axis,mass*size*size/6)
        for axis in ['ixy','ixz','iyz']:add(inertia,axis,0)
        for kind in ['collision','visual']:
            element=add(link,kind,name=kind);add(add(add(element,'geometry'),'box'),'size',f'{size} {size} {size}')
            if kind=='visual':
                material=add(element,'material')
                for tag in ['ambient','diffuse']:add(material,tag,' '.join(map(str,cube['color'])))
            else:
                surface=add(element,'surface');ode=add(add(surface,'friction'),'ode')
                for tag in ['mu','mu2']:add(ode,tag,config['friction'])
                contact=add(add(surface,'contact'),'ode');add(contact,'kp',config['contact_kp']);add(contact,'kd',config['contact_kd'])
    sensors=add(world,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors');add(sensors,'render_engine','ogre2')
    camera=config['camera'];model=add(world,'model',name='fixed_camera');add(model,'static','true');add(model,'pose',' '.join(map(str,camera_pose(config))))
    link=add(model,'link',name=camera['frame']);sensor=add(link,'sensor',name='rgbd',type='rgbd_camera')
    add(sensor,'always_on','true');add(sensor,'update_rate',camera['rate']);add(sensor,'topic','/camera')
    add(sensor,'gz_frame_id',camera['optical_frame']);cam=add(sensor,'camera');add(cam,'optical_frame_id',camera['optical_frame']);add(cam,'horizontal_fov',camera['horizontal_fov'])
    img=add(cam,'image');add(img,'width',camera['width']);add(img,'height',camera['height']);add(img,'format','R8G8B8')
    clip=add(cam,'clip');add(clip,'near',camera['near']);add(clip,'far',camera['far'])
    for camera_node in world.findall('gui/plugin/camera_pose'):camera_node.text='1.7 -1.7 1.6 0 0.4 2.35619449'
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)


def bridge_config(config,path):
    # One RGB-D sensor shares optical center, resolution and intrinsics for both images.
    mappings=[]
    for ros,gz,kind in [('color/image_raw','image','Image'),('depth/image_raw','depth_image','Image'),('color/camera_info','camera_info','CameraInfo'),('depth/camera_info','camera_info','CameraInfo')]:
        mappings.append({'ros_topic_name':'/camera/'+ros,'gz_topic_name':'/camera/'+gz,'ros_type_name':'sensor_msgs/msg/'+kind,
                         'gz_type_name':'gz.msgs.'+kind,'direction':'GZ_TO_ROS','qos_profile':'SENSOR_DATA'})
    Path(path).write_text(yaml.safe_dump(mappings))


def gripper_geometry(model,angles=(0.,.45,.65)):
    """Measure collision mesh inner faces in the installed model, not TF-origin distance."""
    import numpy as np
    import struct
    def rotation(axis,angle):
        axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis);x,y,z=axis
        k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)
    def origin(element):
        t=np.eye(4)
        if element is not None:
            t[:3,3]=list(map(float,element.get('xyz','0 0 0').split()))
            r,p,y=map(float,element.get('rpy','0 0 0').split())
            t[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
        return t
    rows=[]
    for q in angles:
        poses={'robotiq_85_base_link':np.eye(4)}
        for _ in range(len(model.findall('joint'))):
            for j in model.findall('joint'):
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in poses or child in poses:continue
                t=origin(j.find('origin'))
                if j.get('type')!='fixed':
                    mimic=j.find('mimic');angle=q*(float(mimic.get('multiplier','1')) if mimic is not None else 1.)
                    t[:3,:3]=t[:3,:3]@rotation(list(map(float,j.find('axis').get('xyz').split())),angle)
                poses[child]=poses[parent]@t
        bounds={}
        for side in ['left','right']:
            name='robotiq_85_'+side+'_finger_tip_link';collision=model.find("link[@name='%s']/collision"%name)
            mesh=collision.find('geometry/mesh');blob=Path(mesh.get('filename').removeprefix('file://')).read_bytes()
            n=struct.unpack('<I',blob[80:84])[0];points=[]
            if len(blob)!=84+n*50:raise ValueError('Expected installed binary STL')
            for i in range(n):
                values=struct.unpack('<12fH',blob[84+i*50:134+i*50]);points.extend([values[3:6],values[6:9],values[9:12]])
            t=poses[name]@origin(collision.find('origin'));v=(t[:3,:3]@np.array(points).T).T+t[:3,3]
            edge=v[:,0].min() if side=='left' else v[:,0].max();face=v[np.abs(v[:,0]-edge)<1e-5]
            bounds[side]={'min':v.min(axis=0).tolist(),'max':v.max(axis=0).tolist(),'inner_face_z':[float(face[:,2].min()),float(face[:,2].max())]}
        rows.append({'primary_joint_rad':q,'tip_collision_bounds_in_gripper_base':bounds,'inner_gap_m':bounds['left']['min'][0]-bounds['right']['max'][0]})
    return rows


def prepare(folder):
    """Files and startup nodes shared by the full S07 bringup."""
    from ament_index_python.packages import get_package_share_directory as share
    from arm_moveit.scene import load,generate_world
    from launch_ros.actions import Node
    import xacro
    package=Path(share('arm_simulation'));ur=Path(share('ur_simulation_gz'))
    data=yaml.safe_load((package/'config/tabletop.yaml').read_text());(folder/'tabletop.yaml').write_text(yaml.safe_dump(data))
    common=load(Path(share('arm_moveit'))/'config/scene.yaml');world=folder/'world.sdf'
    generate_world(common,'/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf',world);extend_world(world,data)
    for cube in data['cubes']:
        common['objects'].append({**cube,'size':[data['cube_size']]*3,'static':False,'pose_tolerance':.001})
    (folder/'scene.yaml').write_text(yaml.safe_dump(common))
    config=yaml.safe_load((ur/'config/ur_controllers.yaml').read_text())
    config['controller_manager']['ros__parameters']['gripper_controller']={'type':'joint_trajectory_controller/JointTrajectoryController'}
    config['gripper_controller']={'ros__parameters':{'joints':['robotiq_85_left_knuckle_joint'],'command_interfaces':['position'],'state_interfaces':['position','velocity'],
       'constraints':{'goal_time':2.,'stopped_velocity_tolerance':.02,'robotiq_85_left_knuckle_joint':{'trajectory':.1,'goal':.01}}}}
    controllers=folder/'controllers.yaml';controllers.write_text(yaml.safe_dump(config))
    description_file=package/'urdf/ur5e_gripper.urdf.xacro'
    description=xacro.process_file(str(description_file),mappings={'name':'ur','ur_type':'ur5e','safety_limits':'true','simulation_controllers':str(controllers)}).toxml()
    bridge_config(data,folder/'bridge.yaml');pose=camera_pose(data);cam=data['camera']
    from launch.actions import ExecuteProcess
    import sys
    initializer=ExecuteProcess(cmd=[sys.executable,'-m','arm_simulation.tabletop','--init-gripper',str(folder)],output='screen')
    nodes=[initializer,Node(package='controller_manager',executable='spawner',arguments=['gripper_controller','-c','/controller_manager','--controller-manager-timeout','60'],output='screen'),
       Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(folder/'bridge.yaml')}],output='screen'),
       Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--x',str(pose[0]),'--y',str(pose[1]),'--z',str(pose[2]),'--roll',str(pose[3]),'--pitch',str(pose[4]),'--yaw',str(pose[5]),'--frame-id','world','--child-frame-id',cam['frame']],output='screen'),
       Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--roll','-1.57079632679','--pitch','0','--yaw','-1.57079632679','--frame-id',cam['frame'],'--child-frame-id',cam['optical_frame']],output='screen')]
    return {'world':str(world),'description_file':str(description_file),'description':description,'controllers_file':str(controllers),'nodes':nodes,'initializer':initializer}


def extend_moveit(moveit,description):
    """Use identical URDF, extend official SRDF with a measured TCP and gripper group."""
    moveit.robot_description['robot_description']=description
    srdf=ET.fromstring(moveit.robot_description_semantic['robot_description_semantic'])
    group=srdf.find("group[@name='ur_manipulator']")
    for child in list(group):group.remove(child)
    add(group,'chain',base_link='base_link',tip_link='grasp_tcp')
    gripper=add(srdf,'group',name='gripper');add(gripper,'joint',name='robotiq_85_left_knuckle_joint')
    add(srdf,'end_effector',name='robotiq',parent_link='tool0',group='gripper',parent_group='ur_manipulator')
    from ament_index_python.packages import get_package_share_directory as share
    settings=yaml.safe_load((Path(share('arm_simulation'))/'config/tabletop.yaml').read_text())['gripper']
    for name,value in [('open',settings['open']),('closed',settings['closed'])]:add(add(srdf,'group_state',name=name,group='gripper'),'joint',name='robotiq_85_left_knuckle_joint',value=str(value))
    robot=ET.fromstring(description)
    for joint in robot.findall('joint'):
        if joint.get('name').startswith('robotiq_'):
            add(srdf,'disable_collisions',link1=joint.find('parent').get('link'),link2=joint.find('child').get('link'),reason='Adjacent')
    add(srdf,'disable_collisions',link1='wrist_3_link',link2='robotiq_85_base_link',reason='Adjacent mount through fixed flange/tool0')
    # The mimic tree breaks each physical four-bar loop at the inner-link/tip pivot.
    # Actual GetStateValidity contacts identify these two mechanical connections.
    for side in ['left','right']:
        add(srdf,'disable_collisions',link1='robotiq_85_'+side+'_inner_knuckle_link',
            link2='robotiq_85_'+side+'_finger_tip_link',reason='Adjacent closed-loop pivot represented by mimic')
    moveit.robot_description_semantic['robot_description_semantic']=ET.tostring(srdf,encoding='unicode')
    manager=moveit.trajectory_execution['moveit_simple_controller_manager'];manager['controller_names'].append('gripper_controller')
    manager['gripper_controller']={'action_ns':'follow_joint_trajectory','type':'FollowJointTrajectory','default':True,'joints':['robotiq_85_left_knuckle_joint']}



def initialize_gripper(folder):
    """Move off the exact hard stop through the controller after physics is running."""
    import json
    import time
    import traceback
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data,qos_profile_action_status_default
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from rosidl_runtime_py.convert import message_to_ordereddict as serial
    settings=yaml.safe_load((folder/'tabletop.yaml').read_text())['gripper']
    session=json.loads((folder/'session.json').read_text());report={'status':'FAIL','session_id':session['session_id']}
    rclpy.init();node=rclpy.create_node('gripper_initializer');positions={};description=[]
    node.create_subscription(JointState,'/joint_states',lambda m:positions.update(zip(m.name,m.position)),qos_profile_sensor_data)
    node.create_subscription(String,'/robot_description',lambda m:description.append(m.data),qos_profile_action_status_default)
    action=ActionClient(node,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
    controllers=node.create_client(ListControllers,'/controller_manager/list_controllers')
    def wait(predicate,seconds=60):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise TimeoutError('Gripper initialization deadline')
            rclpy.spin_once(node,timeout_sec=.05)
    def result(future):wait(future.done,30);return future.result()
    try:
        wait(lambda:action.server_is_ready() and controllers.service_is_ready() and description)
        def active():
            value=result(controllers.call_async(ListControllers.Request()))
            return any(c.name=='gripper_controller' and c.state=='active' for c in value.controller)
        wait(active)
        name='robotiq_85_left_knuckle_joint'
        goal=FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=[name],points=[
            JointTrajectoryPoint(positions=[settings['open']],velocities=[0.],time_from_start=Duration(sec=settings['move_duration_sec']))]))
        handle=result(action.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Initialization goal rejected')
        value=result(handle.get_result_async());report['result']=serial(value.result)
        if value.status!=4 or value.result.error_code!=0:raise RuntimeError('Initialization motion failed')
        mimics={j.get('name'):float(j.find('mimic').get('multiplier','1')) for j in ET.fromstring(description[-1]).findall('joint') if j.find('mimic') is not None}
        wait(lambda:abs(positions.get(name,99)-settings['open'])<.002 and all(abs(positions.get(n,99)-m*positions[name])<.005 for n,m in mimics.items()),10)
        report.update(status='PASS',actual_positions=dict(positions),open_target=settings['open'])
        temp=folder/'gripper_ready.tmp';temp.write_text(json.dumps(report,indent=2));temp.replace(folder/'gripper_ready.json')
        print(json.dumps(report),flush=True);return 0
    except Exception:
        report['error']=traceback.format_exc();print(report['error'],flush=True);return 1
    finally:
        (folder/'gripper_initialization.json').write_text(json.dumps(report,indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--init-gripper',type=Path,required=True)
    raise SystemExit(initialize_gripper(parser.parse_args().init_gripper))
