"""Pure health rules: no sleeping, ROS calls, or motion commands."""
def evaluate(config, state, now, phase):
    fresh = config['stale_timeout_sec']
    checks = {
        'clock': state.get('clock_progress_at') is not None and now-state['clock_progress_at'] <= fresh,
        'joints': state.get('joints_at') is not None and now-state['joints_at'] <= fresh,
        'tf': state.get('tf_ok', False),
        'controllers': state.get('controllers_at') is not None and now-state['controllers_at'] <= fresh
                       and all(state.get('controllers', {}).get(n) == 'active' for n in config['required_controllers']),
        'gazebo_model': state.get('gazebo_model', False),
    }
    if config.get('camera_required'):
        checks['camera'] = state.get('camera_ok', False)
        checks['gripper_initialized'] = state.get('gripper_initialized',False)
    if phase != 'base':
        checks['moveit'] = state.get('moveit_ready', False)
    return all(checks.values()), checks


class CameraHealth:
    """Keep small metadata histories, never image buffers, and check a common capture."""
    def __init__(self, config):
        from collections import deque
        self.config = config
        self.frames = {k: deque(maxlen=20) for k in ('color', 'depth', 'color_info', 'depth_info')}

    def clear(self):
        for frames in self.frames.values():
            frames.clear()

    def observe(self, key, msg, now):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        c = self.config
        valid = (msg.width == c['width'] and msg.height == c['height']
                 and msg.header.frame_id == c['optical_frame'])
        if key.endswith('info'):
            import math
            expected = c['width'] / (2 * math.tan(c['horizontal_fov'] / 2))
            valid = valid and all(math.isfinite(v) for v in msg.k)
            valid = valid and abs(msg.k[0]-expected) < .01 and abs(msg.k[4]-expected) < .01
            valid = valid and abs(msg.k[2]-c['width']/2) < 1 and abs(msg.k[5]-c['height']/2) < 1
            extra = tuple(msg.k)
        else:
            encoding, size = ('rgb8', 3) if key == 'color' else ('32FC1', 4)
            valid = valid and msg.encoding == encoding and msg.step >= msg.width * size
            valid = valid and len(msg.data) == msg.step * msg.height
            extra = None
        frames=self.frames[key]
        # CameraInfo can repeat one capture many times; retain distinct captures.
        for previous in list(frames):
            if previous[0]==stamp:frames.remove(previous)
        frames.append((stamp, now, bool(valid), extra))

    def ready(self, now, sim_time, tf_ok):
        self.detail = {'sim_time':sim_time,'tf':tf_ok,'latest':{k:{'stamp':v[-1][0],'wall_age':now-v[-1][1],'valid':v[-1][2]} if v else None for k,v in self.frames.items()}}
        if not tf_ok or any(not frames for frames in self.frames.values()):
            return False
        age = self.config['max_age_sec']
        # Reject malformed/latest stale data even when an older good tuple remains.
        if any(not f[-1][2] or now-f[-1][1] > age for f in self.frames.values()):
            return False
        maps = {k: {v[0]: v for v in frames if v[2] and now-v[1] <= age
                    and 0 <= sim_time-v[0] <= age} for k, frames in self.frames.items()}
        common = set.intersection(*(set(m) for m in maps.values()))
        self.detail['common_captures']=len(common)
        if not common:self.detail['capture_history']={k:[v[0] for v in frames] for k,frames in self.frames.items()}
        return any(maps['color_info'][t][3] == maps['depth_info'][t][3] for t in common)
