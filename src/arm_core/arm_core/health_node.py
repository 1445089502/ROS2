"""Condition-based startup gates and continuous read-only simulation health."""
import argparse
import json
import signal
from pathlib import Path
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_action_status_default
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Clock
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener
from arm_core.preflight import load
from arm_core.readiness import evaluate, CameraHealth

JOINTS = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--phase', choices=['base', 'moveit', 'monitor'], required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load(args.config)
    folder = Path(args.session_dir)
    session = json.loads((folder/'session.json').read_text())
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    stopping=False
    def request_shutdown(signum,frame):
        nonlocal stopping
        stopping=True
    previous_handlers={sig:signal.signal(sig,request_shutdown) for sig in (signal.SIGINT,signal.SIGTERM)}
    node = rclpy.create_node('health_node' if args.phase == 'monitor' else args.phase+'_ready_gate',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {}
    subscriptions = []
    camera_health = None
    if config['camera_required']:
        import yaml
        camera_health = CameraHealth(yaml.safe_load((folder/'tabletop.yaml').read_text())['camera'])
        for key, topic, kind in [('color', 'color/image_raw', Image), ('depth', 'depth/image_raw', Image),
                                 ('color_info', 'color/camera_info', CameraInfo), ('depth_info', 'depth/camera_info', CameraInfo)]:
            subscriptions.append(node.create_subscription(kind, '/camera/'+topic,
                lambda m, k=key: camera_health.observe(k, m, time.monotonic()), qos_profile_sensor_data))
    start = time.monotonic()

    def clock(msg):
        value = msg.clock.sec+msg.clock.nanosec/1e9
        if value < state.get('clock_value', value) and camera_health:
            camera_health.clear()
        if value > state.get('clock_value', value):
            state['clock_progress_at'] = time.monotonic()
        state['clock_value'] = value

    def joints(msg):
        if JOINTS.issubset(set(msg.name)) and len(msg.name) == len(msg.position):
            state['joints_at'] = time.monotonic()

    subscriptions.append(node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String, '/robot_description', lambda m:
        state.update(gazebo_model='gz_ros2_control/GazeboSimSystem' in m.data), qos_profile_action_status_default))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    planner = ActionClient(node, MoveGroup, '/move_action')
    execution = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    status_pub = node.create_publisher(String, '/arm/health', qos_profile_action_status_default)
    ready_pub = node.create_publisher(Bool, '/arm/ready', qos_profile_action_status_default)
    future = None
    query_started = 0.
    next_query, next_report = 0., 0.
    last_signature = None
    ever_ready = False
    ready_since = None
    exit_code = 0
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if future is not None and future.done():
                if future.exception() is None:
                    state['controllers'] = {c.name: c.state for c in future.result().controller}
                    state['controllers_at'] = now
                future = None
            elif future is not None and now-query_started > 3.:
                # A dropped startup service response must not freeze readiness forever.
                controllers.remove_pending_request(future)
                future.cancel()
                future = None
                state.pop('controllers_at', None)
                next_query = now+.5
                node.get_logger().warning('Controller query timed out; invalidating stale state and retrying')
            if future is None and now >= next_query and controllers.service_is_ready():
                future = controllers.call_async(ListControllers.Request())
                query_started = now
                next_query = now+.5
            try:
                tf = buffer.lookup_transform(config['base_frame'], config['tool_frame'], rclpy.time.Time())
                stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
                state['tf_ok'] = abs(state.get('clock_value', -1000)-stamp) <= config['stale_timeout_sec']
            except Exception:
                state['tf_ok'] = False
            if camera_health:
                state['gripper_initialized']=(folder/'gripper_ready.json').exists()
                camera_tf = buffer.can_transform(config['base_frame'], camera_health.config['optical_frame'], rclpy.time.Time())
                state['camera_ok'] = camera_health.ready(now, state.get('clock_value', -1000), camera_tf)
            state['moveit_ready'] = planner.server_is_ready() and execution.server_is_ready()
            ready, checks = evaluate(config, state, now, args.phase)
            # Require stable observed readiness; the timer is never the sole readiness condition.
            ready_since = (ready_since if ready_since is not None else now) if ready else None
            stable = ready_since is not None and now-ready_since >= .5
            report = {key: session[key] for key in ['session_id', 'task_id', 'scenario_id', 'config_version']}
            report.update(phase=args.phase, status='READY' if stable else 'WAITING' if not ever_ready else 'DEGRADED',
                          checks=checks, camera=('READY' if state.get('camera_ok') else 'WAITING') if camera_health else 'NOT_REQUIRED', use_sim_time=True,
                          elapsed_wall_sec=round(now-start, 3))
            if camera_health:report['camera_detail']=camera_health.detail
            signature = (report['status'], tuple(checks.items()))
            if now >= next_report or signature != last_signature:
                encoded = json.dumps(report)
                if args.phase == 'monitor':
                    status_pub.publish(String(data=encoded))
                    ready_pub.publish(Bool(data=stable))
                (folder/(args.phase+'_health.json')).write_text(encoded)
                with (folder/'health.jsonl').open('a') as log:
                    log.write(encoded+'\n')
                if signature != last_signature:
                    node.get_logger().info(encoded)
                last_signature = signature
                next_report = now+1.
            if stable:
                ever_ready = True
                if args.phase != 'monitor':
                    break
            if not ever_ready and now-start > config['startup_timeout_sec']:
                node.get_logger().error('Readiness deadline exceeded: '+json.dumps(report))
                exit_code = 2
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig,handler in previous_handlers.items(): signal.signal(sig,handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
