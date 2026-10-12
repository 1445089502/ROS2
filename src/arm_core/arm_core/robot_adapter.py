"""MoveIt planning/execution behind a simulation-only asynchronous adapter."""
from abc import ABC, abstractmethod
import math
import time
from xml.etree import ElementTree as ET

from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from rosidl_runtime_py.convert import message_to_ordereddict
from arm_core.task_policy import TaskError, validate_joints, validate_workspace


class RobotAdapter(ABC):
    @abstractmethod
    async def execute(self, context, request):
        pass

    @abstractmethod
    async def stop(self):
        pass


class RealRobotAdapter(RobotAdapter):
    def __init__(self,*args,**kwargs):
        raise TaskError('REAL_HARDWARE_DISABLED')

    async def execute(self,context,request):
        raise TaskError('REAL_HARDWARE_DISABLED')

    async def stop(self):
        raise TaskError('REAL_HARDWARE_DISABLED')


class GazeboUrAdapter(RobotAdapter):
    def __init__(self,node,config):
        self.node,self.config=node,config
        self.planner=ActionClient(node,MoveGroup,'/move_action')
        self.executor=ActionClient(node,FollowJointTrajectory,config['trajectory_action'])
        self.fk=node.create_client(GetPositionFK,'/compute_fk')
        self.validity=node.create_client(GetStateValidity,'/check_state_validity')
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,node)
        self.pending=self.handle=self.result_future=None
        self.limits={};self.homes={};self.model_loaded=False
        self.execution_count=0
        self.scene_guard=None
        if node.session['scenario_id'] in ['planning_scene','tabletop']:
            from arm_moveit.guard import SceneGuard
            self.scene_guard=SceneGuard(node)

    def close(self):
        if self.scene_guard: self.scene_guard.close()

    async def ensure_scene(self,ctx):
        if self.scene_guard:
            from arm_moveit.scene import SceneError
            try: await self.scene_guard.ensure(ctx,self)
            except SceneError as exc: raise TaskError('SCENE_MISMATCH',str(exc)) from exc

    def load_model(self):
        if self.model_loaded: return
        root=ET.fromstring(self.node.urdf);srdf=ET.fromstring(self.node.srdf)
        for state in srdf.findall('group_state'):
            if state.get('group')==self.config['planning_group']:
                self.homes[state.get('name')]={j.get('name'):float(j.get('value')) for j in state.findall('joint')}
        if self.config['home_name'] not in self.homes: raise TaskError('HOME_NOT_IN_MODEL')
        # Select the arm system by its SRDF home joints, not every control block.
        # S07 adds a separate six-joint gripper block containing five limit-free mimics.
        controlled=set(self.homes[self.config['home_name']])
        systems=[system for system in root.findall('ros2_control')
                 if {j.get('name') for j in system.findall('joint')}==controlled]
        if len(controlled)!=6 or len(systems)!=1: raise TaskError('INVALID_MODEL')
        if systems[0].findtext('hardware/plugin')!='gz_ros2_control/GazeboSimSystem': raise TaskError('NOT_GAZEBO')
        limits={}
        for j in root.findall('joint'):
            if j.get('name') not in controlled: continue
            limit=j.find('limit');soft=j.find('safety_controller')
            if limit is None or j.find('mimic') is not None: raise TaskError('INVALID_MODEL')
            try:
                lo,hi=float(limit.get('lower')),float(limit.get('upper'))
                velocity=float(limit.get('velocity'))
                if soft is not None:
                    lo=max(lo,float(soft.get('soft_lower_limit',lo)));hi=min(hi,float(soft.get('soft_upper_limit',hi)))
            except (TypeError,ValueError) as exc: raise TaskError('INVALID_MODEL') from exc
            if not all(math.isfinite(v) for v in (lo,hi,velocity)) or lo>=hi or velocity<=0: raise TaskError('INVALID_MODEL')
            limits[j.get('name')]={'lower':lo,'upper':hi,'velocity':velocity}
        if set(limits)!=controlled: raise TaskError('INVALID_MODEL')
        self.limits=limits
        self.model_loaded=True

    def require_ready(self):
        now=time.monotonic()
        if not self.node.urdf or not self.node.srdf: raise TaskError('MODEL_NOT_READY')
        self.load_model()
        if now-self.node.joints_at > self.config['state_max_age_sec'] or now-self.node.health_at > self.config['state_max_age_sec']:
            raise TaskError('STATE_STALE',f'joints_age={now-self.node.joints_at:.3f}, health_age={now-self.node.health_at:.3f}')
        if self.node.health.get('session_id')!=self.node.session['session_id'] or self.node.health.get('status')!='READY':
            raise TaskError('NOT_READY')
        if not all(n in self.node.positions for n in self.limits): raise TaskError('JOINT_STATE_INCOMPLETE')
        if not self.planner.server_is_ready() or not self.executor.server_is_ready(): raise TaskError('ACTION_UNAVAILABLE')
        if not self.fk.service_is_ready() or not self.validity.service_is_ready(): raise TaskError('PLANNING_SERVICES_UNAVAILABLE')
        try:
            tf=self.buffer.lookup_transform(self.config['base_frame'],self.config['tool_frame'],Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            if abs(self.node.sim_time-stamp)>self.config['state_max_age_sec']: raise TaskError('TF_STALE')
        except TaskError: raise
        except Exception as exc: raise TaskError('TF_UNAVAILABLE',str(exc))

    def state(self,positions):
        return RobotState(joint_state=JointState(name=list(positions),position=list(positions.values())),is_diff=True)

    async def wait_future(self,future,ctx,timeout=20.):
        end=time.monotonic()+timeout
        while not future.done():
            ctx.check()
            if time.monotonic()>end: raise TaskError('DEPENDENCY_TIMEOUT')
            await self.node.pause()
        ctx.check()
        return future.result()

    async def check_position(self,positions,ctx):
        validate_joints(positions,self.limits)
        state=self.state(positions)
        req=GetPositionFK.Request(header=Header(frame_id=self.config['base_frame']),fk_link_names=[self.config['tool_frame']],robot_state=state)
        result=await self.wait_future(self.fk.call_async(req),ctx)
        if result.error_code.val!=1 or len(result.pose_stamped)!=1: raise TaskError('FK_FAILED')
        p=result.pose_stamped[0].pose.position
        validate_workspace([p.x,p.y,p.z],self.config)
        return state

    async def action(self,client,goal,ctx,phase):
        self.handle=self.result_future=None
        self.pending=client.send_goal_async(goal)
        self.handle=await self.wait_future(self.pending,ctx)
        if not self.handle.accepted:
            self.pending=None
            raise TaskError('ACTION_REJECTED',phase)
        self.result_future=self.handle.get_result_async()
        result=await self.wait_future(self.result_future,ctx,timeout=self.config['max_task_timeout_sec'])
        self.pending=self.handle=self.result_future=None
        return result

    async def execute(self,ctx,data):
        self.require_ready()
        await self.ensure_scene(ctx)
        goal=MoveGroup.Goal();request=goal.request
        request.group_name=self.config['planning_group']; request.pipeline_id='ompl'
        request.num_planning_attempts=self.config['planning_attempts'];request.allowed_planning_time=self.config['planning_time_sec']
        request.max_velocity_scaling_factor=data['velocity_scale'];request.max_acceleration_scaling_factor=data['acceleration_scale']
        request.start_state.is_diff=True
        request.workspace_parameters.header.frame_id=self.config['base_frame']
        for point,values in [(request.workspace_parameters.min_corner,self.config['workspace_min']),(request.workspace_parameters.max_corner,self.config['workspace_max'])]:
            point.x,point.y,point.z=values
        constraints=Constraints(name=ctx.task_id)
        if data['task_type'] in [1,2]:
            target=dict(zip(data['joint_names'],data['joint_positions'])) if data['task_type']==1 else self.homes[self.config['home_name']]
            state=await self.check_position(target,ctx)
            valid=await self.wait_future(self.validity.call_async(GetStateValidity.Request(robot_state=state,group_name=request.group_name)),ctx)
            if not valid.valid: raise TaskError('GOAL_IN_COLLISION')
            constraints.joint_constraints=[JointConstraint(joint_name=n,position=v,tolerance_above=.001,tolerance_below=.001,weight=1.) for n,v in target.items()]
        else:
            pose=data['target_pose']  # Timestamp checked at task admission.
            p=Pose();p.position.x,p.position.y,p.position.z=pose['position'];p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=pose['orientation']
            constraints.position_constraints=[PositionConstraint(header=Header(frame_id=pose['frame_id']),link_name=self.config['tool_frame'],
                constraint_region=BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.002,.002,.002])],primitive_poses=[p]),weight=1.)]
            constraints.orientation_constraints=[OrientationConstraint(header=Header(frame_id=pose['frame_id']),link_name=self.config['tool_frame'],orientation=p.orientation,
                absolute_x_axis_tolerance=.01,absolute_y_axis_tolerance=.01,absolute_z_axis_tolerance=.01,weight=1.)]
        request.goal_constraints=[constraints]
        goal.planning_options.plan_only=True;goal.planning_options.planning_scene_diff.is_diff=True;goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        ctx.phase='PLANNING';ctx.progress=.1
        started=time.monotonic()
        planned=await self.action(self.planner,goal,ctx,'planning')
        ctx.record('planning',{'wall_sec':time.monotonic()-started,'status':planned.status,'error_code':planned.result.error_code.val})
        if planned.status!=4 or planned.result.error_code.val!=1:
            code={-6:'PLANNING_TIMEOUT',-10:'START_IN_COLLISION',-12:'GOAL_IN_COLLISION',-31:'NO_IK_SOLUTION'}.get(planned.result.error_code.val,'PLANNING_FAILED')
            raise TaskError(code,str(planned.result.error_code.val))
        trajectory=planned.result.planned_trajectory
        names=trajectory.joint_trajectory.joint_names;points=trajectory.joint_trajectory.points
        if not points or set(names)!=set(self.limits): raise TaskError('INVALID_PLANNED_TRAJECTORY')
        previous=-1.;path_states=[]
        for point in points:
            stamp=point.time_from_start.sec+point.time_from_start.nanosec/1e9
            if stamp<=previous or not math.isfinite(stamp): raise TaskError('INVALID_TRAJECTORY_TIME')
            previous=stamp
            target=dict(zip(names,point.positions));validate_joints(target,self.limits)
            if len(point.positions)!=len(names) or len(point.velocities)!=len(names) or len(point.accelerations)!=len(names): raise TaskError('TRAJECTORY_FIELDS_MISSING')
            for n,v,a in zip(names,point.velocities,point.accelerations):
                if not math.isfinite(v) or abs(v)>self.limits[n]['velocity']*data['velocity_scale']+1e-3: raise TaskError('TRAJECTORY_SPEED_LIMIT')
                if not math.isfinite(a) or abs(a)>self.config['max_joint_acceleration']: raise TaskError('TRAJECTORY_ACCELERATION_LIMIT')
            if self.scene_guard: path_states.append(target)
            else: await self.check_position(target,ctx)
        if self.scene_guard:
            # Keep in-flight requests below the default service queue depth (10).
            # Validate every trajectory point without overflowing the server queue.
            # Slow trajectories contain many points; serial 50-ms polls would consume the task deadline.
            for offset in range(0,len(path_states),4):
                pending=[]
                for target in path_states[offset:offset+4]:
                    state=self.state(target)
                    fk_request=GetPositionFK.Request(header=Header(frame_id=self.config['base_frame']),fk_link_names=[self.config['tool_frame']],robot_state=state)
                    pending.append((self.fk.call_async(fk_request),self.validity.call_async(GetStateValidity.Request(robot_state=state,group_name=request.group_name))))
                for fk_future,valid_future in pending:
                    result=await self.wait_future(fk_future,ctx)
                    if result.error_code.val!=1 or len(result.pose_stamped)!=1: raise TaskError('FK_FAILED')
                    p=result.pose_stamped[0].pose.position;validate_workspace([p.x,p.y,p.z],self.config)
                    if not (await self.wait_future(valid_future,ctx)).valid: raise TaskError('PATH_IN_COLLISION')
            ctx.record('path_checked',{'trajectory_points':len(path_states),'all_points_collision_free':True})
        await self.ensure_scene(ctx)
        # Never execute a trajectory planned from a materially changed start state.
        self.require_ready()
        if max(abs(self.node.positions[n]-v) for n,v in zip(names,points[0].positions))>.02: raise TaskError('START_STATE_CHANGED')
        ctx.record('trajectory',message_to_ordereddict(trajectory))
        ctx.phase='EXECUTING';ctx.progress=.4
        ctx.target=dict(zip(names,points[-1].positions))
        ctx.start={n:self.node.positions[n] for n in names}
        ctx.check(); self.execution_count+=1
        # Own the controller goal: this fixed MoveIt version accepts cancellation without stopping.
        executed=await self.action(self.executor,FollowJointTrajectory.Goal(trajectory=trajectory.joint_trajectory),ctx,'execution')
        ctx.record('execution',{'status':executed.status,'error_code':executed.result.error_code,'execution_count':self.execution_count})
        if executed.status!=4 or executed.result.error_code!=0: raise TaskError('EXECUTION_FAILED',str(executed.result.error_code))
        self.require_ready()
        if max(abs(self.node.positions[n]-v) for n,v in ctx.target.items())>.01: raise TaskError('TARGET_NOT_REACHED')
        ctx.progress=1.
        ctx.record('actual_final_joints',self.node.positions)

    async def stop(self):
        """Cancel even a late-accepted goal; do not unlock until actual stop is observed."""
        end=time.monotonic()+self.config['stop_timeout_sec']
        if self.pending is not None:
            while not self.pending.done():
                if time.monotonic()>end: raise TaskError('STOP_UNCONFIRMED','Goal acceptance still pending')
                await self.node.pause()
            self.handle=self.pending.result()
            if self.handle.accepted:
                if self.result_future is None: self.result_future=self.handle.get_result_async()
                if not self.result_future.done():
                    task_id=self.node.active.task_id if self.node.active else 'stop'
                    self.node.record(task_id,'downstream_cancel_requested',{'positions':dict(self.node.positions)})
                    cancel=self.handle.cancel_goal_async()
                    while not cancel.done() or not self.result_future.done():
                        if time.monotonic()>end: raise TaskError('STOP_UNCONFIRMED','Action did not reach a terminal state')
                        await self.node.pause()
                    response=cancel.result();terminal=self.result_future.result()
                    self.node.record(task_id,'downstream_cancel_result',{'return_code':response.return_code,'goals_canceling':len(response.goals_canceling),'status':terminal.status})
                    if terminal.status not in (4,5,6): raise TaskError('STOP_UNCONFIRMED','Downstream result is not terminal')
        self.pending=self.handle=self.result_future=None
        stable_since=None;anchor=None
        while time.monotonic()<end:
            now=time.monotonic()
            fresh=now-self.node.joints_at<self.config['state_max_age_sec']
            slow=bool(self.limits) and all(n in self.node.velocities and abs(self.node.velocities[n])<self.config['stop_speed_rad_sec'] for n in self.limits)
            if fresh and slow:
                if stable_since is None:
                    stable_since=now;anchor={n:self.node.positions[n] for n in self.limits}
                if max(abs(self.node.positions[n]-anchor[n]) for n in self.limits)>.005:
                    stable_since=None
                elif now-stable_since>=self.config['stop_stable_sec']:
                    return
            else: stable_since=None
            await self.node.pause()
        raise TaskError('STOP_UNCONFIRMED','No fresh stable joint state; manager remains locked')
