"""Recheck both scenes before planning and immediately before execution."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from moveit_msgs.srv import GetPlanningScene
from arm_moveit.scene import load,scene_request,verify_moveit,gazebo_snapshot,SceneError


class SceneGuard:
    def __init__(self,node):
        self.node=node;self.folder=node.folder;self.data=load(self.folder/'scene.yaml')
        ready=json.loads((self.folder/'scene_ready.json').read_text())
        if ready['session_id']!=node.session['session_id'] or ready['config_sha256']!=hashlib.sha256((self.folder/'scene.yaml').read_bytes()).hexdigest():
            raise SceneError('SCENE_NOT_READY')
        self.query=node.create_client(GetPlanningScene,'/get_planning_scene')
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='gazebo_scene')
        self.counter=0

    async def ensure(self,ctx,adapter):
        self.counter+=1;tag=ctx.task_id+'-'+str(self.counter)
        future=self.worker.submit(gazebo_snapshot,self.data,self.folder,tag)
        while not future.done():
            ctx.check();await self.node.pause()
        actual=future.result()
        response=await adapter.wait_future(self.query.call_async(scene_request()),ctx)
        planned=verify_moveit(self.data,response.scene)
        # Physical carried objects require S07b; S05 fixture is only for planning tests.
        if response.scene.robot_state.attached_collision_objects:
            raise SceneError('ATTACHED_OBJECT_EXECUTION_NOT_ENABLED_S05')
        ctx.record('scene_confirmed',{'gazebo':actual,'moveit':planned,'snapshot':tag})

    def close(self):
        self.worker.shutdown(wait=True)
