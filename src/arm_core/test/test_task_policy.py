import copy
import json
import math
from pathlib import Path
import pytest
import yaml
from arm_core.task_policy import Ledger, TaskError, validate_request, validate_workspace, validate_joints

CONFIG={'max_velocity_scale':.1,'max_acceleration_scale':.1,'max_task_timeout_sec':180.,'base_frame':'base_link',
        'workspace_min':[-1.,-1.,.02],'workspace_max':[1.,1.,1.4]}


def goal():
    return {'task_id':'test-1','scene_reset_id':'scene-1','target_id':'','task_type':1,
            'joint_names':['j'+str(i) for i in range(6)],'joint_positions':[.1]*6,
            'target_pose':{'frame_id':'','stamp':0.,'position':[0.,0.,0.],'orientation':[0.,0.,0.,0.]},
            'velocity_scale':.1,'acceleration_scale':.1,'timeout_sec':30.}


def test_valid_and_order_independent_fingerprint():
    first=goal();second=copy.deepcopy(first)
    second['joint_names'].reverse();second['joint_positions'].reverse()
    assert validate_request(first,CONFIG)==validate_request(second,CONFIG)


@pytest.mark.parametrize('field,value,code',[
    ('task_id','../bad','INVALID_TASK_ID'),('task_type',99,'INVALID_TASK_TYPE'),
    ('velocity_scale',1.,'SPEED_LIMIT'),('acceleration_scale',0.,'SPEED_LIMIT'),
    ('timeout_sec',0.,'INVALID_TIMEOUT'),('timeout_sec',181.,'INVALID_TIMEOUT'),
    ('timeout_sec',math.nan,'NONFINITE_PARAMETER'),('scene_reset_id','','INVALID_SCENE_OR_TARGET_ID'),
    ('joint_positions',[0.,math.inf,0.,0.,0.,0.],'INVALID_JOINT_TARGET'),
    ('joint_names',['same']*6,'INVALID_JOINT_TARGET')])
def test_request_rejection(field,value,code):
    data=goal();data[field]=value
    with pytest.raises(TaskError) as error: validate_request(data,CONFIG)
    assert error.value.code==code


def test_pose_frame_quaternion_and_workspace():
    data=goal();data.update(task_type=3,joint_names=[],joint_positions=[])
    data['target_pose']={'frame_id':'camera','stamp':1.,'position':[0.,0.,.5],'orientation':[0.,0.,0.,1.]}
    with pytest.raises(TaskError,match='INVALID_FRAME'):validate_request(data,CONFIG)
    data['target_pose']['frame_id']='base_link';data['target_pose']['orientation']=[0.]*4
    with pytest.raises(TaskError,match='INVALID_POSE'):validate_request(data,CONFIG)
    data['target_pose']['orientation']=[0.,0.,0.,1.];data['target_pose']['position']=[2.,0.,.5]
    with pytest.raises(TaskError,match='OUTSIDE_WORKSPACE'):validate_request(data,CONFIG)


def test_joint_limits_and_names():
    limits={'j0':{'lower':-1.,'upper':1.}}
    validate_joints({'j0':.5},limits)
    with pytest.raises(TaskError,match='JOINT_NAMES_MISMATCH'):validate_joints({'bad':0.},limits)
    with pytest.raises(TaskError):validate_joints({'j0':1.01},limits)


def test_completed_ledger_record_survives_restart(tmp_path):
    path=tmp_path/'tasks.sqlite';ledger=Ledger(path)
    ledger.add('a','hash','scene');ledger.update('a','RUNNING');ledger.update('a','SUCCEEDED');ledger.db.close()
    ledger=Ledger(path);assert ledger.get('a')['state']=='SUCCEEDED';ledger.db.close()


def test_inflight_ledger_is_not_reexecuted_after_restart(tmp_path):
    path=tmp_path/'tasks.sqlite';ledger=Ledger(path)
    ledger.add('a','hash','scene');ledger.update('a','RUNNING');ledger.db.close()
    ledger=Ledger(path);assert ledger.get('a')['error_code']=='SERVER_RESTART';assert ledger.get('a')['state']=='ABORTED';ledger.db.close()


def test_terminal_state_cannot_be_reopened(tmp_path):
    ledger=Ledger(tmp_path/'tasks.sqlite');ledger.add('a','hash','scene');ledger.update('a','RUNNING');ledger.update('a','SUCCEEDED')
    with pytest.raises(TaskError,match='SUCCEEDED -> RUNNING'):ledger.update('a','RUNNING')
    ledger.db.close()


def test_canceling_cannot_report_success(tmp_path):
    ledger=Ledger(tmp_path/'tasks.sqlite');ledger.add('a','hash','scene');ledger.update('a','CANCELING')
    with pytest.raises(TaskError):ledger.update('a','SUCCEEDED')
    ledger.update('a','CANCELED');ledger.db.close()


def test_unused_pose_nan_is_explained_instead_of_json_crash():
    data=goal();data['target_pose']['position'][0]=math.nan
    with pytest.raises(TaskError,match='NONFINITE_PARAMETER'):validate_request(data,CONFIG)


def test_real_ros_node_construction_with_isolated_ledger(tmp_path):
    import rclpy
    from arm_core.task_manager import TaskManager
    config=Path(__file__).resolve().parents[2]/'arm_bringup/config/motion.yaml'
    (tmp_path/'session.json').write_text(json.dumps({'session_id':'smoke-session','scenario_id':'bare_arm','config_version':'s02.1'}))
    rclpy.init(args=[])
    node=None
    try:
        node=TaskManager(tmp_path,config,ledger_path=tmp_path/'isolated.sqlite3')
        assert node.get_name()=='task_manager'
        assert node.scene_reset_id=='smoke-session'
        assert node.adapter.execution_count==0
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_durable_worker_yields_and_retains_terminal_result(tmp_path):
    import asyncio
    from arm_core.task_policy import AsyncLedger
    ticks=[]
    async def pause():
        ticks.append(1)
        await asyncio.sleep(.001)
    ledger=AsyncLedger(tmp_path/'async.sqlite',pause)
    async def run():
        await ledger.add('durable','hash','scene')
        await ledger.update('durable','RUNNING')
        await ledger.update('durable','SUCCEEDED')
        assert ledger.get('durable')['state']=='SUCCEEDED'
    try:
        asyncio.run(run())
        assert ticks  # The caller yielded while disk writes ran.
    finally: ledger.close()
    reopened=Ledger(tmp_path/'async.sqlite')
    assert reopened.get('durable')['state']=='SUCCEEDED'
    assert reopened.db.execute('PRAGMA synchronous').fetchone()[0]==2
    reopened.db.close()


def test_arm_model_ignores_separate_gripper_mimics():
    from arm_core.robot_adapter import GazeboUrAdapter
    from types import SimpleNamespace
    names=['joint_'+str(i) for i in range(6)]
    joints=''.join('<joint name="'+n+'"><limit lower="-2" upper="2" velocity="1"/></joint>' for n in names)
    control='<ros2_control><hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>'+''.join('<joint name="'+n+'"/>' for n in names)+'</ros2_control>'
    extra='<ros2_control><joint name="gripper"/><joint name="follower"/></ros2_control><joint name="follower"><mimic joint="gripper"/></joint>'
    srdf='<robot><group_state name="up" group="ur_manipulator">'+''.join('<joint name="'+n+'" value="0"/>' for n in names)+'</group_state></robot>'
    adapter=GazeboUrAdapter.__new__(GazeboUrAdapter)
    adapter.node=SimpleNamespace(urdf='<robot>'+joints+control+extra+'</robot>',srdf=srdf)
    adapter.config={'planning_group':'ur_manipulator','home_name':'up'}
    adapter.model_loaded=False;adapter.limits={};adapter.homes={}
    adapter.load_model()
    assert set(adapter.limits)==set(names)
    assert adapter.model_loaded
