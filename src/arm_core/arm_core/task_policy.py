"""Pure request validation and durable idempotency; no ROS imports or motion."""
import hashlib
import json
import math
import re
import sqlite3
import time
from concurrent.futures import ThreadPoolExecutor

TERMINAL = {'SUCCEEDED', 'ABORTED', 'CANCELED'}


class TaskError(Exception):
    def __init__(self, code, message=''):
        super().__init__(message or code)
        self.code = code


def finite(values):
    return all(isinstance(x, (int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in values)


def validate_request(data, config):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', data['task_id']):
        raise TaskError('INVALID_TASK_ID')
    if data['task_type'] not in [1,2,3,4]:
        raise TaskError('INVALID_TASK_TYPE')
    if not data['scene_reset_id'] or len(data['scene_reset_id']) > 96 or len(data['target_id']) > 96:
        raise TaskError('INVALID_SCENE_OR_TARGET_ID')
    if not finite([data[k] for k in ['velocity_scale','acceleration_scale','timeout_sec']]):
        raise TaskError('NONFINITE_PARAMETER')
    if not 0 < data['velocity_scale'] <= config['max_velocity_scale'] or not 0 < data['acceleration_scale'] <= config['max_acceleration_scale']:
        raise TaskError('SPEED_LIMIT')
    if not 0 < data['timeout_sec'] <= config['max_task_timeout_sec']:
        raise TaskError('INVALID_TIMEOUT')
    pose=data['target_pose']
    if not finite([pose['stamp']]+pose['position']+pose['orientation']):
        raise TaskError('NONFINITE_PARAMETER')
    names, positions = data['joint_names'], data['joint_positions']
    if data['task_type'] == 1:
        if len(names) != 6 or len(set(names)) != 6 or len(positions) != 6 or not finite(positions):
            raise TaskError('INVALID_JOINT_TARGET')
    elif names or positions:
        raise TaskError('UNUSED_JOINT_FIELDS')
    if data['task_type'] == 3:
        pose=data['target_pose']; q=pose['orientation']; xyz=pose['position']
        if pose['frame_id'] != config['base_frame']:
            raise TaskError('INVALID_FRAME')
        if not finite(xyz+q) or abs(sum(v*v for v in q)-1) > .001:
            raise TaskError('INVALID_POSE')
        validate_workspace(xyz,config)
    # Canonicalize named joints: order is not semantic identity.
    value = dict(data)
    if names:
        pairs=sorted(zip(names,positions)); value['joint_names']=[p[0] for p in pairs]; value['joint_positions']=[p[1] for p in pairs]
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def validate_workspace(xyz, config):
    if not finite(xyz) or not all(lo <= x <= hi for x,lo,hi in zip(xyz,config['workspace_min'],config['workspace_max'])):
        raise TaskError('OUTSIDE_WORKSPACE')


def validate_joints(target, limits):
    if set(target) != set(limits):
        raise TaskError('JOINT_NAMES_MISMATCH')
    for name,value in target.items():
        if not finite([value]) or not limits[name]['lower'] <= value <= limits[name]['upper']:
            raise TaskError('JOINT_LIMIT',name)


class Ledger:
    def __init__(self,path):
        self.db=sqlite3.connect(str(path))
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('CREATE TABLE IF NOT EXISTS tasks (task_id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL, state TEXT NOT NULL, code TEXT NOT NULL, message TEXT NOT NULL, scene TEXT NOT NULL, updated REAL NOT NULL)')
        self.db.execute("UPDATE tasks SET state='ABORTED', code='SERVER_RESTART', message='Previous process ended before a confirmed terminal state' WHERE state NOT IN ('SUCCEEDED','ABORTED','CANCELED')")
        self.db.commit()

    def get(self,task_id):
        row=self.db.execute('SELECT task_id,fingerprint,state,code,message,scene FROM tasks WHERE task_id=?',(task_id,)).fetchone()
        return dict(zip(['task_id','fingerprint','state','error_code','message','scene_reset_id'],row)) if row else None

    def add(self,task_id,fingerprint,scene):
        self.db.execute('INSERT INTO tasks VALUES (?,?,?,?,?,?,?)',(task_id,fingerprint,'ACCEPTED','','',scene,time.time()))
        self.db.commit()

    def update(self,task_id,state,code='',message=''):
        previous=self.get(task_id)
        allowed={'ACCEPTED':{'RUNNING','CANCELING','ABORTED'}, 'RUNNING':{'CANCELING','SUCCEEDED','ABORTED'}, 'CANCELING':{'CANCELED','ABORTED'}}
        if previous['state'] != state and state not in allowed.get(previous['state'],set()):
            raise TaskError('INVALID_STATE_TRANSITION',previous['state']+' -> '+state)
        self.db.execute('UPDATE tasks SET state=?,code=?,message=?,updated=? WHERE task_id=?',(state,code,message,time.time(),task_id))
        self.db.commit()


class AsyncLedger:
    """One DB-owning worker; ROS callbacks read snapshots without waiting for fsync.

    Accepted IDs are reserved before yielding. Motion starts only after the
    durable add/RUNNING writes finish; failed writes lock out new motion.
    """
    def __init__(self,path,pause):
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='task_ledger')
        self.ledger=self.worker.submit(Ledger,path).result()
        def snapshot():
            ids=[r[0] for r in self.ledger.db.execute('SELECT task_id FROM tasks')]
            return {task_id:self.ledger.get(task_id) for task_id in ids}
        self.rows=self.worker.submit(snapshot).result()
        self.pause=pause;self.failed=False;self.closed=False

    def get(self,task_id):
        row=self.rows.get(task_id)
        return dict(row) if row else None

    async def write(self,method,*args):
        future=self.worker.submit(method,*args)
        while not future.done(): await self.pause()
        try: return future.result()
        except Exception as exc:
            self.failed=True
            for row in self.rows.values():
                if row['state'] not in TERMINAL:
                    row.update(state='ABORTED',error_code='LEDGER_IO_ERROR',message=str(exc))
            raise TaskError('LEDGER_IO_ERROR',str(exc)) from exc

    async def add(self,task_id,fingerprint,scene):
        self.rows[task_id]={'task_id':task_id,'fingerprint':fingerprint,'scene_reset_id':scene,
                            'state':'ACCEPTED','error_code':'','message':''}
        try: await self.write(self.ledger.add,task_id,fingerprint,scene)
        except TaskError as exc:
            self.rows[task_id].update(state='ABORTED',error_code=exc.code,message=str(exc))
            raise

    async def update(self,task_id,state,code='',message=''):
        await self.write(self.ledger.update,task_id,state,code,message)
        self.rows[task_id].update(state=state,error_code=code,message=message)

    def close(self):
        if not self.closed:
            self.worker.submit(self.ledger.db.close).result()
            self.worker.shutdown();self.closed=True
