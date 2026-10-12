"""One-shot gate: actual Gazebo geometry -> apply MoveIt -> read back -> ready."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import traceback
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from arm_moveit.scene import load,gazebo_snapshot,scene_diff,scene_request,verify_moveit,SceneError,close_values


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--session-dir',type=Path,required=True)
    args,ros_args=parser.parse_known_args();folder=args.session_dir
    data=load(folder/'scene.yaml');session=json.loads((folder/'session.json').read_text())
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('scene_setup',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    report={'status':'FAIL','session_id':session['session_id']}
    def wait(predicate,seconds=30):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline: raise TimeoutError('Scene setup deadline')
            rclpy.spin_once(node,timeout_sec=.05)
    def call(client,request):
        wait(client.service_is_ready);future=client.call_async(request);wait(future.done);return future.result()
    try:
        wait(lambda:buffer.can_transform('world',data['frame'],Time()))
        tf=buffer.lookup_transform('world',data['frame'],Time()).transform
        if not close_values([tf.translation.x,tf.translation.y,tf.translation.z,tf.rotation.x,tf.rotation.y,tf.rotation.z,abs(tf.rotation.w)],[0,0,0,0,0,0,1]):
            raise SceneError('WORLD_BASE_TRANSFORM_NOT_IDENTITY')
        report['gazebo']=gazebo_snapshot(data,folder,'bootstrap')
        apply=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        query=node.create_client(GetPlanningScene,'/get_planning_scene')
        if not call(apply,ApplyPlanningScene.Request(scene=scene_diff(data))).success: raise SceneError('SCENE_APPLY_FAILED')
        report['moveit']=verify_moveit(data,call(query,scene_request()).scene)
        report.update(status='PASS',config_sha256=hashlib.sha256((folder/'scene.yaml').read_bytes()).hexdigest())
        (folder/'scene_ready.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        return 0
    except Exception:
        report['error']=traceback.format_exc();print(report['error'],flush=True);return 1
    finally:
        (folder/'scene_setup_report.json').write_text(json.dumps(report,indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
