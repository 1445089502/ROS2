from pathlib import Path
from xml.etree import ElementTree as ET
import copy
import pytest
import yaml
from arm_moveit.scene import load,generate_world,verify_sdf,SceneError,scene_diff,verify_moveit

CONFIG=Path(__file__).resolve().parents[1]/'config/scene.yaml'


def test_same_dimensions_in_physics_visual_and_planning(tmp_path):
    data=load(CONFIG);base=tmp_path/'base.sdf';base.write_text('<!-- tutorial: gz service --req -->\n<sdf version="1.6"><world name="empty"/></sdf>')
    generated=tmp_path/'world.sdf';generate_world(data,base,generated)
    assert set(verify_sdf(data,generated.read_text()))=={'table','wall','obstacle'}
    assert set(verify_moveit(data,scene_diff(data)))=={'table','wall','obstacle'}
    tree=ET.parse(generated);tree.find('.//model/link/collision/geometry/box/size').text='9 9 9'
    with pytest.raises(SceneError,match='SIZE_MISMATCH'):verify_sdf(data,ET.tostring(tree.getroot(),encoding='unicode'))


@pytest.mark.parametrize('field,value',[('size',[0,1,1]),('position',[float('nan'),0,0]),('id','../bad')])
def test_invalid_scene_rejected(tmp_path,field,value):
    data=yaml.safe_load(CONFIG.read_text());data['objects'][0][field]=value
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(data))
    with pytest.raises(SceneError):load(path)


def test_moved_or_missing_moveit_object_rejected():
    data=load(CONFIG);scene=scene_diff(data)
    scene.world.collision_objects[0].primitive_poses[0].position.x+=.05
    with pytest.raises(SceneError,match='POSE_MISMATCH'):verify_moveit(data,scene)
    scene=scene_diff(data);scene.world.collision_objects.pop()
    with pytest.raises(SceneError,match='MISSING'):verify_moveit(data,scene)
