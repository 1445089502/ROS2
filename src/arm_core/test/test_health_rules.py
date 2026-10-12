from copy import deepcopy
from pathlib import Path
import pytest
from arm_core.preflight import load, validate_config, inspect_environment
from arm_core.readiness import evaluate

CONFIG = load(Path(__file__).resolve().parents[2]/'arm_bringup/config/sim.yaml')


def healthy():
    return dict(clock_progress_at=10., joints_at=10., controllers_at=10., tf_ok=True,
                controllers={n: 'active' for n in CONFIG['required_controllers']},
                gazebo_model=True, moveit_ready=True)


@pytest.mark.parametrize('key,value', [('real_hardware', True), ('use_sim_time', False),
    ('startup_timeout_sec', -1), ('stale_timeout_sec', float('nan')),
    ('required_controllers', ['wrong_controller']), ('camera_required', True)])
def test_reject_unsafe_or_unsupported_configuration(key, value):
    config = deepcopy(CONFIG)
    config[key] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_unsourced_environment_is_detected_before_ros_import():
    with pytest.raises(ValueError, match='not sourced'):
        inspect_environment(CONFIG, environ={})


def test_missing_package_is_named():
    def missing(name):
        raise LookupError(name)
    with pytest.raises(ValueError, match='ur_simulation_gz'):
        inspect_environment(CONFIG, environ={'ROS_DISTRO': 'jazzy', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
            'ROS_DOMAIN_ID': '91', 'GZ_PARTITION': 'ros2_s01_ur5e'}, resolver=missing)


def test_system_moveit_without_fix_is_rejected():
    with pytest.raises(ValueError, match='shutdown fix'):
        inspect_environment(CONFIG, environ={'ROS_DISTRO': 'jazzy', 'AMENT_PREFIX_PATH': '/opt/ros/jazzy',
            'ROS_DOMAIN_ID': '91', 'GZ_PARTITION': 'ros2_s01_ur5e'}, resolver=lambda _: '/opt/ros/jazzy')


def test_ready_does_not_depend_on_elapsed_startup_sleep():
    assert evaluate(CONFIG, healthy(), 10.1, 'monitor')[0]
    assert not evaluate(CONFIG, {}, 1000., 'monitor')[0]


@pytest.mark.parametrize('field,value', [('clock_progress_at', 0.), ('joints_at', 0.),
    ('controllers_at', 0.), ('tf_ok', False), ('moveit_ready', False), ('gazebo_model', False),
    ('controllers', {'joint_state_broadcaster': 'active'})])
def test_loss_of_required_component_is_unhealthy(field, value):
    state = healthy()
    state[field] = value
    assert not evaluate(CONFIG, state, 10.1, 'monitor')[0]


def test_base_gate_does_not_require_moveit_before_it_is_started():
    state = healthy()
    state['moveit_ready'] = False
    assert evaluate(CONFIG, state, 10.1, 'base')[0]
    assert not evaluate(CONFIG, state, 10.1, 'moveit')[0]


def test_tabletop_requires_camera_and_gripper():
    config = load(Path(__file__).resolve().parents[2]/'arm_bringup/config/tabletop.yaml')
    state = healthy()
    assert not evaluate(config, state, 10.1, 'base')[0]
    state['controllers']['gripper_controller'] = 'active'
    assert not evaluate(config, state, 10.1, 'base')[0]
    state['camera_ok'] = True
    assert not evaluate(config, state, 10.1, 'monitor')[0]
    state['gripper_initialized'] = True
    assert evaluate(config, state, 10.1, 'monitor')[0]


@pytest.mark.parametrize('fault', ['none', 'missing', 'stale', 'future', 'unsynced', 'frame', 'encoding', 'intrinsics', 'tf', 'rollback', 'repeated_info'])
def test_camera_capture_gate(fault):
    from arm_core.readiness import CameraHealth
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    import math
    config = dict(width=4, height=2, optical_frame='optical', horizontal_fov=1., max_age_sec=2.)
    camera = CameraHealth(config)
    fx = 4/(2*math.tan(.5))
    for key in camera.frames:
        if fault == 'missing' and key == 'depth':
            continue
        header = Header(stamp=Time(sec=10), frame_id='optical')
        if fault == 'future': header.stamp.sec = 12
        if fault == 'unsynced' and key == 'depth': header.stamp.sec = 9
        if fault == 'frame' and key == 'depth': header.frame_id = 'wrong'
        if key.endswith('info'):
            msg = CameraInfo(header=header, width=4, height=2, k=[fx,0.,2.,0.,fx,1.,0.,0.,1.])
            if fault == 'intrinsics': msg.k[0] = 0.
        else:
            size = 3 if key == 'color' else 4
            msg = Image(header=header, width=4, height=2, encoding='rgb8' if size == 3 else '32FC1', step=4*size, data=[0]*(8*size))
            if fault == 'encoding': msg.encoding = '16UC1'
        camera.observe(key, msg, 7. if fault == 'stale' else 10.)
    if fault == 'repeated_info':
        # Repeated newer calibration must not evict a still-fresh aligned capture.
        for key in ('color_info','depth_info'):
            msg=CameraInfo(header=Header(stamp=Time(sec=11),frame_id='optical'),width=4,height=2,k=[fx,0.,2.,0.,fx,1.,0.,0.,1.])
            for _ in range(50):camera.observe(key,msg,10.05)
    if fault == 'rollback': camera.clear()
    assert camera.ready(10.1, 10.1, fault != 'tf') == (fault in ('none','repeated_info'))
