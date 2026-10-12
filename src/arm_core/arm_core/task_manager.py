"""Task lifecycle + durable idempotency. Every long operation yields to ROS callbacks."""
import argparse
import fcntl
import json
import signal
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_action_status_default
from rclpy.task import Future
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from arm_interfaces.action import ExecuteArmTask
from arm_interfaces.srv import GetArmTask
from arm_core.task_policy import AsyncLedger, TaskError, TERMINAL, validate_request
from arm_core.robot_adapter import GazeboUrAdapter


def request_data(request):
    p=request.target_pose
    return {'task_id':request.task_id,'scene_reset_id':request.scene_reset_id,'target_id':request.target_id,
            'task_type':request.task_type,'joint_names':list(request.joint_names),'joint_positions':list(request.joint_positions),
            'target_pose':{'frame_id':p.header.frame_id,'stamp':p.header.stamp.sec+p.header.stamp.nanosec/1e9,
                'position':[p.pose.position.x,p.pose.position.y,p.pose.position.z],
                'orientation':[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]},
            'velocity_scale':request.velocity_scale,'acceleration_scale':request.acceleration_scale,'timeout_sec':request.timeout_sec}


class Context:
    def __init__(self,manager,handle):
        self.manager,self.handle=manager,handle
        self.task_id=handle.request.task_id
        self.deadline=time.monotonic()+handle.request.timeout_sec
        self.cancel=False;self.phase='VALIDATING';self.progress=0.;self.target=self.start=None

    def check(self):
        if self.cancel or self.handle.is_cancel_requested: raise TaskError('CANCELED')
        if time.monotonic()>self.deadline: raise TaskError('TASK_TIMEOUT')
        self.manager.adapter.require_ready()

    def record(self,event,value):
        self.manager.record(self.task_id,event,value)


class TaskManager(Node):
    def __init__(self,folder,config_path,ledger_path=None):
        super().__init__('task_manager',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.folder=folder;self.folder.mkdir(parents=True,exist_ok=True)
        self.session=json.loads((folder/'session.json').read_text())
        self.config=yaml.safe_load(config_path.read_text())
        if self.config['adapter']!='gazebo_ur' or self.config['real_hardware'] is not False:
            raise TaskError('REAL_HARDWARE_DISABLED')
        (folder/'motion.yaml').write_bytes(config_path.read_bytes())
        self.scene_reset_id=self.session['session_id']
        self.event_log=(folder/'tasks.jsonl').open('a',buffering=1)
        self.ledger=AsyncLedger(ledger_path or Path.home()/'.ros/arm_tasks.sqlite3',self.pause)
        self.positions={};self.velocities={};self.health={};self.urdf=self.srdf=None
        self.joints_at=self.health_at=0.;self.sim_time=0.
        self.shutting_down=False;self.stopping_tasks=0
        self.active=None;self.fault=None;self.waiters=[];self.last_feedback=0.
        self._task_subscriptions=[self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data),
            self.create_subscription(String,'/robot_description',lambda m:setattr(self,'urdf',m.data),qos_profile_action_status_default),
            self.create_subscription(String,'/robot_description_semantic',lambda m:setattr(self,'srdf',m.data),qos_profile_action_status_default),
            self.create_subscription(String,'/arm/health',self.on_health,qos_profile_action_status_default),
            self.create_subscription(ClockMessage,'/clock',lambda m:setattr(self,'sim_time',m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data)]
        self.adapter=GazeboUrAdapter(self,self.config)
        group=ReentrantCallbackGroup()
        self.server=ActionServer(self,ExecuteArmTask,'/arm/execute_task',execute_callback=self.execute,
            goal_callback=lambda request:GoalResponse.REJECT if self.shutting_down else GoalResponse.ACCEPT,cancel_callback=self.cancel,callback_group=group,result_timeout=300)
        self.service=self.create_service(GetArmTask,'/arm/get_task',self.get_task,callback_group=group)
        # Steady clock keeps cancellation/deadlines responsive even if Gazebo /clock pauses.
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.record('bootstrap','manager_started',{'scene_reset_id':self.scene_reset_id})

    def destroy_node(self):
        self.adapter.close()
        self.ledger.close()
        self.event_log.close()
        return super().destroy_node()

    def on_joints(self,msg):
        self.joints_at=time.monotonic();self.positions=dict(zip(msg.name,msg.position));self.velocities=dict(zip(msg.name,msg.velocity))

    def on_health(self,msg):
        self.health_at=time.monotonic();self.health=json.loads(msg.data)

    async def pause(self):
        future=Future();self.waiters.append(future);await future

    def tick(self):
        waiters,self.waiters=self.waiters,[]
        for future in waiters:
            if not future.done(): future.set_result(None)
        now=time.monotonic()
        ctx=self.active
        if ctx and now-self.last_feedback>=.2:
            if ctx.target:
                total=max(abs(ctx.target[n]-ctx.start[n]) for n in ctx.target)
                remaining=max(abs(ctx.target[n]-self.positions.get(n,ctx.start[n])) for n in ctx.target)
                ctx.progress=max(ctx.progress,min(.95,.4+.55*(1-remaining/max(total,.001))))
            state=self.ledger.get(ctx.task_id)['state']
            ctx.handle.publish_feedback(ExecuteArmTask.Feedback(task_id=ctx.task_id,state=state,phase=ctx.phase,progress=float(ctx.progress)))
            self.last_feedback=now

    def record(self,task_id,event,data):
        value={'session_id':self.session['session_id'],'task_id':task_id,'scene_reset_id':self.scene_reset_id,
               'scenario_id':self.session['scenario_id'],'config_version':self.config['config_version'],
               'wall_monotonic':time.monotonic(),'sim_time':self.sim_time,'event':event,'data':data}
        self.event_log.write(json.dumps(value)+'\n')

    async def transition(self,task_id,state,code='',message=''):
        await self.ledger.update(task_id,state,code,message);self.record(task_id,'transition',{'state':state,'error_code':code,'message':message})

    def cancel(self,handle):
        if self.active and self.active.task_id==handle.request.task_id:
            self.active.cancel=True
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def get_task(self,request,response):
        row=self.ledger.get(request.task_id) if request.task_id else None
        response.found=row is not None;response.task_id=request.task_id
        if row:
            response.state=row['state'];response.error_code=row['error_code'];response.message=row['message']
        response.active_task_id=self.active.task_id if self.active else ''
        response.scene_reset_id=self.scene_reset_id;response.session_id=self.session['session_id']
        response.execution_count=self.adapter.execution_count
        try:
            self.adapter.require_ready();response.ready=not self.shutting_down and self.fault is None and not self.ledger.failed
        except TaskError: response.ready=False
        if not request.task_id and self.fault:
            response.error_code=self.fault;response.message='Stop unconfirmed; new motion is locked until recovery'
        return response

    def finish(self,handle,state,code='',message='',replayed=False):
        if state=='SUCCEEDED': handle.succeed()
        elif state=='CANCELED' and handle.is_cancel_requested: handle.canceled()
        else: handle.abort()
        return ExecuteArmTask.Result(task_id=handle.request.task_id,state=state,error_code=code,message=message,replayed=replayed)

    async def execute(self,handle):
        data=request_data(handle.request);task_id=data['task_id']
        try:
            if self.shutting_down: raise TaskError('SHUTTING_DOWN')
            if self.ledger.failed: raise TaskError('LEDGER_IO_ERROR')
            fingerprint=validate_request(data,self.config)
            previous=self.ledger.get(task_id)
            if previous:
                if previous['fingerprint']!=fingerprint: raise TaskError('TASK_ID_CONFLICT')
                self.record(task_id,'idempotent_replay',{'state':previous['state']})
                while previous['state'] not in TERMINAL:
                    if handle.is_cancel_requested and self.active and self.active.task_id==task_id: self.active.cancel=True
                    await self.pause();previous=self.ledger.get(task_id)
                return self.finish(handle,previous['state'],previous['error_code'],previous['message'],True)
            if data['scene_reset_id']!=self.scene_reset_id: raise TaskError('SCENE_RESET_MISMATCH')
            if self.fault: raise TaskError(self.fault,'Motion locked after an unconfirmed stop')
            if (self.active or self.stopping_tasks) and data['task_type']!=4: raise TaskError('BUSY',self.active.task_id if self.active else 'STOP')
            self.adapter.require_ready()
            # Freshness is an admission condition; disk/scene checks must not age
            # an already accepted static pose. Scene identity is checked again before motion.
            if data['task_type']==3:
                age=self.sim_time-data['target_pose']['stamp']
                if age<-.1 or age>self.config['target_max_age_sec']: raise TaskError('TARGET_STALE')
        except TaskError as exc:
            self.record(task_id,'rejected',{'error_code':exc.code,'message':str(exc)})
            return self.finish(handle,'ABORTED',exc.code,str(exc))
        # Reserve the single motion slot before persistence yields to callbacks.
        ctx=None
        if data['task_type']!=4:
            ctx=Context(self,handle);self.active=ctx
        else:
            self.stopping_tasks+=1
            if self.active: self.active.cancel=True
        try:
            await self.ledger.add(task_id,fingerprint,self.scene_reset_id)
            self.record(task_id,'accepted',data)
            await self.transition(task_id,'RUNNING')
        except TaskError as exc:
            self.fault=exc.code
            if self.active is ctx: self.active=None
            if data['task_type']==4: self.stopping_tasks-=1
            return self.finish(handle,'ABORTED',exc.code,str(exc))
        if data['task_type']==4:
            try:
                active=self.active
                if active:
                    active.cancel=True
                    while self.active is active:
                        await self.pause()
                    if self.fault:
                        await self.transition(task_id,'ABORTED',self.fault)
                        return self.finish(handle,'ABORTED',self.fault)
                else:
                    try: await self.adapter.stop()
                    except TaskError as exc:
                        self.fault=exc.code;await self.transition(task_id,'ABORTED',exc.code,str(exc))
                        return self.finish(handle,'ABORTED',exc.code,str(exc))
                await self.transition(task_id,'SUCCEEDED')
                return self.finish(handle,'SUCCEEDED')
            except TaskError as exc:
                self.fault=exc.code
                return self.finish(handle,'ABORTED',exc.code,str(exc))
            finally:
                self.stopping_tasks-=1
        try:
            await self.adapter.execute(ctx,data)
            ctx.check()
            await self.transition(task_id,'SUCCEEDED')
            return self.finish(handle,'SUCCEEDED')
        except Exception as exc:
            code=exc.code if isinstance(exc,TaskError) else 'INTERNAL_ERROR'
            ctx.phase='STOPPING'
            try: await self.transition(task_id,'CANCELING',code,str(exc))
            except TaskError as ledger_error:
                self.fault=ledger_error.code;code=ledger_error.code
            try:
                await self.adapter.stop()
                self.record(task_id,'stop_confirmed',{'positions':self.positions,'velocities':self.velocities})
            except Exception as stop_error:
                self.fault='STOP_UNCONFIRMED';code=self.fault;exc=stop_error
            state='CANCELED' if code=='CANCELED' else 'ABORTED'
            try: await self.transition(task_id,state,code,str(exc))
            except TaskError as ledger_error:
                self.fault=ledger_error.code;state='ABORTED';code=ledger_error.code
            return self.finish(handle,state,code,str(exc))
        finally:
            self.active=None


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir',type=Path,required=True);parser.add_argument('--config',type=Path,required=True)
    args,ros_args=parser.parse_known_args(argv)
    lock=(Path.home()/'.ros/arm_task_manager.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Keep the ROS context alive while callbacks finish and cancellation drains.
    # The default rclpy signal handler can invalidate it during take_message().
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=TaskManager(args.session_dir,args.config)
    stop_at=None
    def request_shutdown(signum,frame):
        nonlocal stop_at
        if stop_at is None: stop_at=time.monotonic()
        node.shutting_down=True
        if node.active: node.active.cancel=True
    previous={sig:signal.signal(sig,request_shutdown) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if stop_at is not None:
                if node.active is None: break
                if time.monotonic()-stop_at>node.config['stop_timeout_sec']+3:
                    raise RuntimeError('Shutdown could not confirm active task stop')
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for sig,handler in previous.items(): signal.signal(sig,handler)
        lock.close()



if __name__=='__main__': main()
