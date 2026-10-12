"""One box specification for Gazebo collision/visual geometry and MoveIt."""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from xml.etree import ElementTree as ET
import yaml


class SceneError(ValueError):
    pass


def load(path):
    data=yaml.safe_load(Path(path).read_text())
    if data['version']!='s05.1' or data['frame']!='base_link' or data['world']!='arm_scene':
        raise SceneError('UNSUPPORTED_SCENE')
    ids=[]
    for obj in data['objects']:
        if not re.fullmatch('[a-z][a-z0-9_]*',obj['id']): raise SceneError('INVALID_OBJECT_ID')
        ids.append(obj['id'])
        for field in ['position','size']:
            if len(obj[field])!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in obj[field]):
                raise SceneError('INVALID_GEOMETRY')
        if min(obj['size'])<=0: raise SceneError('INVALID_GEOMETRY')
    if len(ids)!=len(set(ids)): raise SceneError('DUPLICATE_OBJECT_ID')
    return data


def generate_world(data,official_world,destination):
    # Reuse the installed official world's physics/light/ground/plugins.
    # Gazebo accepts its tutorial comments containing --req; strict XML parsers reject '--' in comments.
    source=re.sub(r'<!--.*?-->','',Path(official_world).read_text(),flags=re.S)
    tree=ET.ElementTree(ET.fromstring(source));world=tree.getroot().find('world');world.set('name',data['world'])
    # Reuse the installed GUI plugins; give every new machine a useful view.
    gui_config=Path(official_world).parent.parent/'gui/gui.config'
    if gui_config.exists():
        text=re.sub(r'<\?xml.*?\?>|<!--.*?-->','',gui_config.read_text(),flags=re.S)
        installed=ET.fromstring('<root>'+text+'</root>')
        gui=world.find('gui')
        if gui is not None: world.remove(gui)
        gui=ET.SubElement(world,'gui',fullscreen='false')
        for plugin in installed.findall('plugin'):
            camera=plugin.find('camera_pose')
            if camera is not None:camera.text='2.6 -2.6 2.0 0 0.365 2.35619449'
            gui.append(plugin)
    for obj in data['objects']:
        model=ET.SubElement(world,'model',name=obj['id'])
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,obj['position']+[0,0,0]))
        link=ET.SubElement(model,'link',name='body')
        for kind in ['collision','visual']:
            element=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text=' '.join(map(str,obj['size']))
            if kind=='visual':
                material=ET.SubElement(element,'material')
                for name in ['ambient','diffuse']: ET.SubElement(material,name).text=' '.join(map(str,obj['color']))
    ET.indent(tree);tree.write(destination,encoding='utf-8',xml_declaration=True)
    return hashlib.sha256(Path(destination).read_bytes()).hexdigest()


def close_values(a,b,tolerance=1e-5):
    return len(a)==len(b) and all(math.isfinite(x) and abs(x-y)<=tolerance for x,y in zip(a,b))


def verify_sdf(data,xml):
    world=ET.fromstring(xml).find('world')
    if world is None or world.get('name')!=data['world']: raise SceneError('GAZEBO_WORLD_MISMATCH')
    models={m.get('name'):m for m in world.findall('model')}
    checked={}
    for obj in data['objects']:
        model=models.get(obj['id'])
        if model is None or (model.findtext('static','false') in ['true','1'])!=obj.get('static',True): raise SceneError('GAZEBO_OBJECT_MISSING: '+obj['id'])
        pose=[float(x) for x in model.findtext('pose','0 0 0 0 0 0').split()]
        if not close_values(pose,obj['position']+[0,0,0],obj.get('pose_tolerance',1e-5)): raise SceneError('GAZEBO_POSE_MISMATCH: '+obj['id'])
        links=model.findall('link')
        if len(links)!=1: raise SceneError('GAZEBO_LINK_MISMATCH')
        if not close_values([float(x) for x in links[0].findtext('pose','0 0 0 0 0 0').split()],[0]*6): raise SceneError('GAZEBO_LINK_POSE_MISMATCH')
        for kind in ['collision','visual']:
            elements=links[0].findall(kind)
            if len(elements)!=1: raise SceneError('GAZEBO_GEOMETRY_MISSING')
            size=[float(x) for x in elements[0].findtext('geometry/box/size','').split()]
            if not close_values(size,obj['size']): raise SceneError('GAZEBO_SIZE_MISMATCH: '+obj['id'])
            if not close_values([float(x) for x in elements[0].findtext('pose','0 0 0 0 0 0').split()],[0]*6): raise SceneError('GAZEBO_GEOMETRY_POSE_MISMATCH')
        checked[obj['id']]={'position':pose[:3],'size':obj['size'],'collision_and_visual':True}
    return checked


def gazebo_snapshot(data,folder,tag):
    # CLI uses Gazebo transport, not a fabricated ROS scene echo.
    from google.protobuf import text_format
    try:
        from gz.msgs10.stringmsg_pb2 import StringMsg
    except ImportError:
        sys.path.insert(0,'/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python')
        from gz.msgs10.stringmsg_pb2 import StringMsg
    command=['gz','service','-s','/world/'+data['world']+'/generate_world_sdf',
             '--reqtype','gz.msgs.SdfGeneratorConfig','--reptype','gz.msgs.StringMsg','--timeout','5000','--req','global_entity_gen_config { expand_include_tags { data: true } }']
    result=subprocess.run(command,capture_output=True,text=True,timeout=10)
    (folder/(tag+'-gz-command.json')).write_text(json.dumps({'command':command,'exit_code':result.returncode,'stderr':result.stderr}))
    if result.returncode: raise SceneError('GAZEBO_QUERY_FAILED')
    message=StringMsg();text_format.Parse(result.stdout,message)
    (folder/(tag+'-actual-world.sdf')).write_text(message.data)
    return verify_sdf(data,message.data)


def collision_object(obj,frame):
    from moveit_msgs.msg import CollisionObject
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=obj['position'];pose.orientation.w=1.
    message=CollisionObject(id=obj['id'],operation=CollisionObject.ADD)
    message.header.frame_id=frame;message.pose.orientation.w=1.
    message.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=obj['size'])];message.primitive_poses=[pose]
    return message


def scene_diff(data):
    from moveit_msgs.msg import PlanningScene
    scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
    scene.world.collision_objects=[collision_object(obj,data['frame']) for obj in data['objects']]
    return scene


def scene_request():
    from moveit_msgs.msg import PlanningSceneComponents as C
    from moveit_msgs.srv import GetPlanningScene
    return GetPlanningScene.Request(components=C(components=C.WORLD_OBJECT_GEOMETRY|C.ROBOT_STATE_ATTACHED_OBJECTS|C.ALLOWED_COLLISION_MATRIX))


def verify_moveit(data,scene):
    objects={o.id:o for o in scene.world.collision_objects};checked={}
    for obj in data['objects']:
        value=objects.get(obj['id'])
        if value is None or value.header.frame_id not in [data['frame'],'world']: raise SceneError('PLANNING_SCENE_MISSING: '+obj['id'])
        if len(value.primitives)!=1 or value.primitives[0].type!=1 or len(value.primitive_poses)!=1: raise SceneError('PLANNING_SCENE_GEOMETRY')
        if not close_values(list(value.primitives[0].dimensions),obj['size']): raise SceneError('PLANNING_SCENE_SIZE_MISMATCH')
        a,b=value.pose,value.primitive_poses[0]
        for pose in [a,b]:
            if not close_values([pose.orientation.x,pose.orientation.y,pose.orientation.z,abs(pose.orientation.w)],[0,0,0,1]): raise SceneError('PLANNING_SCENE_ROTATION_MISMATCH')
        xyz=[a.position.x+b.position.x,a.position.y+b.position.y,a.position.z+b.position.z]
        if not close_values(xyz,obj['position']): raise SceneError('PLANNING_SCENE_POSE_MISMATCH: '+obj['id'])
        checked[obj['id']]={'position':xyz,'size':list(value.primitives[0].dimensions)}
    # No blanket allowed collisions with the required obstacles.
    acm=scene.allowed_collision_matrix
    for i,name in enumerate(acm.entry_names):
        if name in objects and name in [o['id'] for o in data['objects']] and any(acm.entry_values[i].enabled):
            raise SceneError('OBSTACLE_COLLISION_DISABLED')
    for name,allowed in zip(acm.default_entry_names,acm.default_entry_values):
        if allowed: raise SceneError('DEFAULT_COLLISION_DISABLED: '+name)
    return checked


def payload_diff(data,attached,world_position=None):
    from moveit_msgs.msg import PlanningScene,AttachedCollisionObject,CollisionObject
    scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True;obj=data['payload']
    attachment=AttachedCollisionObject(link_name=obj['link'],touch_links=obj['touch_links'])
    attachment.object=collision_object(obj,obj['link']);attachment.object.operation=CollisionObject.ADD if attached else CollisionObject.REMOVE
    scene.robot_state.attached_collision_objects=[attachment]
    # MoveIt attachment removes an existing world object itself. An extra REMOVE
    # for this new fixture makes ApplyPlanningScene return false (object absent).
    if not attached and world_position is not None:
        scene.world.collision_objects=[collision_object({**obj,'position':world_position},data['frame'])]
    return scene
