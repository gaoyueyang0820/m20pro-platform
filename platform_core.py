"""Protocol, kinematics, diagnostics, persistence and training. Python 3.9+."""
import copy
import json
import math
import os
import re
import shlex
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

import m20_console as original
from observation_errors import sync_error

ROOT = Path(__file__).resolve().parent
LEGS = ['FL', 'FR', 'HL', 'HR']
PARTS = ['HipX', 'HipY', 'Knee', 'Wheel']
NEUTRAL = [0., -.3, .6, 0., 0., -.3, .6, 0., 0., .3, -.6, 0., 0., .3, -.6, 0.]
JOINTS = []
urdf = ET.parse(ROOT / 'platform_web/assets/model/M20.urdf')
for leg in LEGS:
    for part in PARTS:
        name = '%s_%s_joint' % (leg.lower(), part.lower())
        limit = urdf.find(".//joint[@name='%s']/limit" % name)
        JOINTS.append(dict(index=len(JOINTS), name=name, leg=leg, part=part,
                           minimum=float(limit.get('lower', '-3.14159')),
                           maximum=float(limit.get('upper', '3.14159')),
                           unit='rad/s' if part == 'Wheel' else 'rad'))


def number(value, lo=-1e5, hi=1e5):
    if isinstance(value, bool):
        raise ValueError('数值不能是布尔值')
    try:
        value = float(value)
    except (ValueError, TypeError):
        raise ValueError('请输入有效数值')
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError('数值必须在 %s 到 %s 之间' % (lo, hi))
    return value


def integer(value, lo=0, hi=65535):
    value = number(value, lo, hi)
    if value != int(value):
        raise ValueError('该参数必须为整数')
    return int(value)


def rpc(base, path, data=None, timeout=4, token=None):
    if not base:
        raise ValueError('尚未配置机器人端/工作站代理地址，请在连接设置中配置')
    headers = {'Content-Type': 'application/json'}
    if token:
        headers['X-API-Token'] = token
    req = Request(base.rstrip('/') + path, headers=headers,
                  data=json.dumps(data).encode() if data is not None else None)
    try:
        with urlopen(req, timeout=timeout) as r:
            result = json.load(r)
    except HTTPError as exc:
        try:
            payload=json.loads(exc.read());message=payload.get('err', '代理返回错误')
        except Exception:
            payload=None;message = '代理 HTTP %s' % exc.code
        pending=sync_error(payload,path,data)
        if pending:raise pending
        raise ValueError(message)
    except (URLError, TimeoutError, OSError):
        raise ValueError('代理连接失败或超时，请检查地址、SSH 隧道和代理进程')
    if isinstance(result, dict) and result.get('ok') is False:
        pending=sync_error(result,path,data)
        if pending:raise pending
        raise ValueError(result.get('err', '代理操作失败'))
    return result


def age_of(record):
    """Receive age survives wall-clock corrections; legacy fixtures still work."""
    try:
        if not record or 'monotonic_at' not in record and record.get('at') is None:return math.inf
        age=time.monotonic()-record['monotonic_at'] if 'monotonic_at' in record else time.time()-record.get('at',0)
        return age if math.isfinite(age) and age>=0 else math.inf
    except (TypeError,ValueError):return math.inf


class RobotLink(original.M20Link):
    """Keep the supplied transport; validate frames and key by (Type, Command)."""
    def __init__(self, ip, port=30000):
        self.records = {}
        self.events = []
        self.drive_lock = threading.RLock()
        self.inhibit = False
        self._observation_deadline = None
        self._vel_monotonic=0.;self._zero_until=0.
        super().__init__(ip, port)
        threading.Thread(target=self._poll, daemon=True).start()

    @staticmethod
    def _parse(data):
        if len(data) < 16 or data[:4] != original.SYNC or data[8] != 1:
            return None
        if struct.unpack_from('<H', data, 4)[0] != len(data) - 16:
            return None
        try:
            obj = json.loads(data[16:])
            pd = obj['PatrolDevice']
            if not isinstance(pd, dict) or not isinstance(pd.get('Items'), dict):
                return None
            if not isinstance(pd.get('Type'), int) or not isinstance(pd.get('Command'), int):
                return None
            return obj
        except (ValueError, KeyError, TypeError):
            return None

    def _rx_loop(self):
        while not self._stop:
            try:
                frame, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            if addr[0] != self.ip:
                continue
            msg = self._parse(frame)
            if not msg:
                continue
            pd = msg['PatrolDevice']
            now = time.time()
            record = dict(type=pd['Type'], command=pd['Command'], items=pd['Items'],
                          seq=struct.unpack_from('<H', frame, 6)[0], at=now,monotonic_at=time.monotonic())
            with self.lock:
                self.last_rx = now
                self.records['%s:%s' % (pd['Type'], pd['Command'])] = record
                self.events.append(record)
                del self.events[:-500]
                if pd['Type'] == 1002:
                    self.snapshots[str(pd['Command'])] = pd['Items']

    def _poll(self):
        next_slow=0.
        while not self._stop:
            # Pose is a control input. Reading it only once a second made a
            # fresh lidar pose look inconsistent while the robot was walking.
            queries=[(1007,2)];now=time.monotonic()
            if now>=next_slow:
                queries.extend([(1007,1),(2002,1),(1101,7)]);next_slow=now+1
            for t,c in queries:
                if self._stop:
                    return
                self.send(self.apdu(t, c))
            time.sleep(.2)

    def record(self, t, c):
        with self.lock:
            return copy.deepcopy(self.records.get('%s:%s' % (t, c), {}))

    def command(self, t, c, items, label):
        buf = self.apdu(t, c, items)
        seq = struct.unpack_from('<H', buf, 6)[0]
        sent_at=time.time();sent_monotonic=time.monotonic()
        if not self.send(buf, label):
            raise ValueError('UDP 发送失败')
        return dict(seq=seq, sent_at=sent_at,sent_monotonic=sent_monotonic,type=t, command=c,
                    delivery='sent', message='已发送，等待机器人状态或业务结果确认')

    def set_velocity(self, f, s, y, valid_until=None):
        v = dict(f=number(f, -1, 1), s=number(s, -1, 1), y=number(y, -1, 1))
        with self.drive_lock:
            if self.inhibit and any(v.values()):
                raise ValueError('软急停已锁定，请先显式起立恢复')
            with self.lock:
                was_moving=any(self._vel.values())
                self._vel = v
                self._observation_deadline = valid_until if any(v.values()) else None
                self._vel_ts = time.time()
                self._vel_monotonic=time.monotonic()
                if any(v.values()):
                    self._vel_last_nz = self._vel_ts
                    self._zero_until=0.
                elif was_moving:self._zero_until=self._vel_monotonic+.4
            if not any(v.values()):
                self._send_axis(0, 0, 0)
        return True

    def _vel_loop(self):
        while not self._stop:
            time.sleep(.05)
            if self._stop:return
            self._velocity_tick()

    def _velocity_tick(self):
        with self.drive_lock:
            with self.lock:
                v,stamp=dict(self._vel),self._vel_monotonic
                basic_record=self.records.get('1002:6',{});basic=basic_record.get('items',{}).get('BasicStatus',{})
            now=time.monotonic()
            if any(v.values()) and (self.inhibit or now-stamp>.5 or
                    self._observation_deadline is not None and now>=self._observation_deadline or
                    age_of(basic_record)>2 or basic.get('HES')!=0 or basic.get('MotionState')!=17 or
                    basic.get('Sleep') or basic.get('ControlUsageMode') not in [0,2]):
                self.set_velocity(0,0,0);self._log('err','控制心跳/机器人反馈超时，速度已归零')
            elif any(v.values()) or now<self._zero_until and basic.get('ControlUsageMode') in [0,2]:
                self._send_axis(v['f'],v['s'],v['y'])

    def close(self,stop=True):
        if stop:
            self.set_velocity(0, 0, 0)
            # A lost UDP zero packet must not be the only shutdown attempt.
            for _ in range(2):self._send_axis(0,0,0)
        self._stop = True
        self.sock.close()


def load_error_catalog():
    catalog = {}
    path = ROOT.parent / 'M20Pro文档学习/docs/35_错误码与异常处理.md'
    # Also ship a standalone copy, so the deliverable does not need the study workspace.
    if not path.exists():
        path = ROOT / 'reference/errors.md'
    if path.exists():
        for code, dec, title in re.findall(r'\|\s*(0x[0-9a-fA-F]+)\s*\|\s*(\d+)\s*\|\s*([^|\n]+)', path.read_text()):
            catalog[int(dec)] = title.strip()
    return catalog


ERRORS = load_error_catalog()


def explain_error(code, component=0):
    code = int(code, 0) if isinstance(code, str) else int(code)
    title = ERRORS.get(code, '未收录的错误码，请保留原始回包并核对固件文档')
    group = '关节' if 0x8000 <= code < 0x8100 else ('电池' if 0x8100 <= code < 0x8200 else
            '计算单元' if 0x8200 <= code < 0x8300 else '充电' if 0x8500 <= code < 0x8600 else
            '导航' if 0xA000 <= code < 0xB000 or code in [12, 0x8605, 0x2300, 0x2302] else '协议')
    affected = []
    if group == '关节':
        affected = [j['leg'] + '.' + j['part'] for i, j in enumerate(JOINTS) if component & (1 << i)]
    elif group == '电池':
        affected = [n for i, n in enumerate(['后侧电池', '前侧电池']) if component & (1 << i)]
    hint = '检查原始回包、时间戳与固件版本。'
    if '温' in title:
        hint = '停止高负载动作，检查温度趋势和散热，降温后按厂商流程处理。'
    elif any(x in title for x in ['定位', '地图', '雷达']):
        hint = '停止导航，检查雷达点云、活动地图、定位状态及 /ODOM 的更新时间。'
    elif any(x in title for x in ['电池', '电量', '电压', '电流']):
        hint = '检查双电池电压、电量、温度与接线；按厂商电池排障流程处理。'
    elif group == '关节':
        hint = '停止动作并检查对应关节的反馈、线缆和驱动状态；不要反复清错后重试。'
    elif group == '充电':
        hint = '检查充电桩供电、反光柱识别与障碍物，结合 Charge 状态确认。'
    return dict(code=code, hex='0x%04X' % code, title=title, group=group,
                affected=affected, hint=hint, component=component,
                level='info' if code in [0, 0x2300, 0x2302] else 'warning' if
                ('预警' in title or code == 12) else 'error')


def leg_geometry(leg):
    """Geometry for the supplied M20 URDF, including the lateral wheel offset."""
    if leg not in LEGS:raise ValueError('无效腿编号')
    joints=[urdf.find(".//joint[@name='%s_%s_joint']"%(leg.lower(),p)) for p in ['hipx','hipy','knee','wheel']]
    origins=[list(map(float,j.find('origin').get('xyz').split())) for j in joints]
    if [j.find('axis').get('xyz') for j in joints[:3]]!=['-1 0 0','0 -1 0','0 -1 0'] or any(j.find('origin').get('rpy','0 0 0')!='0 0 0' for j in joints):
        raise ValueError('模型关节轴与已核对的 M20 版本不一致')
    return origins[0],-origins[2][2],-origins[3][2],origins[2][1]+origins[3][1]


def leg_forward(positions,leg):
    origin,l1,l2,lateral=leg_geometry(leg)
    i=LEGS.index(leg)*4;abduction,hip,knee=positions[i:i+3]
    x=l1*math.sin(hip)+l2*math.sin(hip+knee)
    z=-l1*math.cos(hip)-l2*math.cos(hip+knee)
    return [origin[0]+x,origin[1]+math.cos(abduction)*lateral+math.sin(abduction)*z,
            origin[2]-math.sin(abduction)*lateral+math.cos(abduction)*z]


def leg_target(positions, leg, height):
    """Raise one wheel centre along body Z, preserving body X/Y and other legs.

    Uses the supplied URDF's link lengths, axes and lateral offsets. This is a
    suspended rig motion; it provides no standing balance controller.
    """
    if leg not in LEGS or len(positions) != 16:
        raise ValueError('无效腿编号或关节反馈')
    height = number(height, 0, .08)
    target = list(positions)
    i = LEGS.index(leg) * 4
    hip, knee = positions[i + 1:i + 3]
    origin,l1,l2,lateral=leg_geometry(leg)
    foot=leg_forward(positions,leg)
    x,y,body_z=[foot[n]-origin[n] for n in range(3)];body_z+=height
    squared=y*y+body_z*body_z-lateral*lateral
    if squared<=0:raise ValueError('目标超出侧摆关节可达范围')
    z=-math.sqrt(squared)
    if z >= -.12:
        raise ValueError('当前姿态不适合单腿抬升，请先在仿真中调整')
    cosine = (x*x + z*z - l1*l1-l2*l2) / (2*l1*l2)
    if not -1 <= cosine <= 1:
        raise ValueError('目标超出腿部可达范围')
    q2 = math.copysign(math.acos(cosine), knee if abs(knee) > .05 else (1 if leg in LEGS[:2] else -1))
    q1 = math.atan2(x, -z) - math.atan2(l2*math.sin(q2), l1+l2*math.cos(q2))
    q0=math.atan2(z,lateral)-math.atan2(body_z,y)
    q0=positions[i]+math.atan2(math.sin(q0-positions[i]),math.cos(q0-positions[i]))
    target[i]=positions[i] if abs(q0-positions[i])<1e-12 else q0
    target[i + 1], target[i + 2] = q1, q2
    for j in [i,i + 1, i + 2]:
        number(target[j], JOINTS[j]['minimum'], JOINTS[j]['maximum'])
    return target


class Store:
    def __init__(self, path=None):
        self.path = Path(path or ROOT / 'data/workspace.json')
        self.lock = threading.RLock()
        self.value = dict(routes=[], maps=[], poses=[], history=[])
        if self.path.exists():
            try:
                self.value.update(json.loads(self.path.read_text()))
            except (ValueError, OSError):
                raise ValueError('工作区数据无法读取，请先恢复 data/workspace.json 备份')

    def save(self):
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix('.tmp')
            tmp.write_text(json.dumps(self.value, ensure_ascii=False, indent=2))
            tmp.replace(self.path)


TASKS = {'rough': 'Rough-Deeprobotics-M20-v0', 'flat': 'Flat-Deeprobotics-M20-v0'}


class Training:
    """Managed argv-only worker. A remote instance runs on the NVIDIA workstation."""
    def __init__(self, demo=False, root=None, python=None):
        self.demo = demo
        self.root = Path(root or os.environ.get('M20_TRAIN_ROOT',
                              ROOT.parent / 'M20Pro代码学习/sources/rl_training')).resolve()
        self.python = python or os.environ.get('M20_TRAIN_PYTHON', sys.executable)
        self.jobs = []
        self.lock = threading.RLock()
        self.processes = {}
        self.output = ROOT / 'data/training'

    def config(self, req):
        task = req.get('task', 'rough')
        if task not in TASKS:
            raise ValueError('训练任务必须是 rough 或 flat')
        kind = req.get('kind', 'train')
        if kind not in ['train', 'play', 'export']:
            raise ValueError('未知训练操作')
        cfg = dict(task=task, kind=kind, envs=integer(req.get('envs', 64), 1, 4096),
                   iterations=integer(req.get('iterations', 5000), 1, 20000),
                   seed=integer(req.get('seed', 42), 0, 2147483647))
        checkpoint = str(req.get('checkpoint', ''))
        if kind != 'train':
            p = (self.root / checkpoint).resolve()
            if self.root not in p.parents or p.suffix != '.pt':
                raise ValueError('检查点必须位于训练工程目录内，且为 .pt 文件')
            cfg['checkpoint'] = str(p)
        script = self.root / 'scripts/reinforcement_learning/rsl_rl' / ('play.py' if kind == 'play' else 'train.py')
        argv = [self.python, '-u', str(script), '--task', TASKS[task], '--num_envs', str(cfg['envs']), '--seed', str(cfg['seed']), '--headless']
        if kind == 'train':
            argv += ['--max_iterations', str(cfg['iterations'])]
        elif kind == 'play':
            argv += ['--checkpoint', cfg['checkpoint']]
        else:
            argv = [self.python, '-u', str(self.root / 'scripts/tools/export_onnx_fast.py'),
                    '--robot', 'm20', '--checkpoint_path', cfg['checkpoint'], '--output_path', 'OUTPUT', '--no_metadata']
        return cfg, argv

    def preview(self, req):
        cfg, argv = self.config(req)
        return dict(ok=True, config=cfg, argv=argv, command=shlex.join(argv), root=str(self.root),
                    mode='demo' if self.demo else 'workstation')

    def start(self, req):
        cfg, argv = self.config(req)
        with self.lock:
            if any(j['status'] in ['running', 'stopping'] for j in self.jobs):
                raise ValueError('已有任务运行中，请先停止或等待完成')
            if not self.demo:
                if not Path(argv[2]).is_file():
                    raise ValueError('找不到训练脚本，请设置 M20_TRAIN_ROOT')
                if cfg['kind'] != 'train' and not Path(cfg['checkpoint']).is_file():
                    raise ValueError('检查点不存在')
            job_id = uuid.uuid4().hex[:10]
            out = self.output / job_id
            out.mkdir(parents=True, exist_ok=True)
            if cfg['kind'] == 'export':
                argv[argv.index('OUTPUT')] = str(out / 'policy.onnx')
            job = dict(id=job_id, config=cfg, argv=argv, command=shlex.join(argv), status='running',
                       started=time.time(), iteration=0, reward=None, metrics=[], logs=[],
                       demo=self.demo, output=str(out))
            (out / 'job.json').write_text(json.dumps(job, ensure_ascii=False, indent=2))
            self.jobs.insert(0, job)
            threading.Thread(target=self._run, args=(job,), daemon=True).start()
            return copy.deepcopy(job)

    def _log(self, job, line):
        with self.lock:
            job['logs'].append(line)
            del job['logs'][:-200]
        with (Path(job['output']) / 'console.log').open('a') as f:
            f.write(line + '\n')

    def _run(self, job):
        try:
            if self.demo:
                self._log(job, '[演示] 合成训练曲线，仅用于验证任务界面，不代表策略效果')
                count = min(job['config']['iterations'], 60)
                for step in range(1, count + 1):
                    time.sleep(.35)
                    with self.lock:
                        if job['status'] != 'running':
                            job['status'] = 'stopped'
                            return
                        job['iteration'] = round(step / count * job['config']['iterations'])
                        job['reward'] = round(18 * (1 - math.exp(-step / 20)) + math.sin(step) * .3, 3)
                        job['metrics'].append(dict(step=job['iteration'], reward=job['reward']))
                    self._log(job, 'Learning iteration %s | Mean reward: %s' % (job['iteration'], job['reward']))
                job['status'] = 'completed'
            else:
                env = os.environ.copy()
                env['PYTHONUNBUFFERED'] = '1'
                # Training never inherits control-plane secrets it does not use.
                for key in ['M20_BRIDGE_TOKEN', 'M20_API_TOKEN']:
                    env.pop(key, None)
                proc = subprocess.Popen(job['argv'], cwd=str(self.root), env=env, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, start_new_session=os.name != 'nt')
                with self.lock:
                    self.processes[job['id']] = proc
                    if job['status'] == 'stopping':
                        proc.terminate()
                for line in proc.stdout:
                    text = re.sub(r'\x1b\[[0-9;]*m', '', line).rstrip()
                    self._log(job, text)
                    iteration = re.search(r'Learning iteration\s+(\d+)', text, re.I)
                    reward = re.search(r'Mean (?:total )?reward:\s*(-?[\d.]+)', text, re.I)
                    with self.lock:
                        if iteration:
                            job['iteration'] = int(iteration.group(1))
                        if reward:
                            job['reward'] = float(reward.group(1))
                            job['metrics'].append(dict(step=job['iteration'], reward=job['reward']))
                            del job['metrics'][:-1000]
                code = proc.wait()
                job['exit_code'] = code
                job['status'] = 'stopped' if job['status'] == 'stopping' else 'completed' if code == 0 else 'failed'
        except Exception as exc:
            self._log(job, '任务失败: ' + str(exc))
            job['status'] = 'failed'
        finally:
            job['ended'] = time.time()
            (Path(job['output']) / 'job.json').write_text(json.dumps(job, ensure_ascii=False, indent=2))

    def stop(self, job_id):
        with self.lock:
            job = next((j for j in self.jobs if j['id'] == job_id), None)
            if not job:
                raise ValueError('任务不存在')
            if job['status'] != 'running':
                return dict(ok=True)
            job['status'] = 'stopping'
            proc = self.processes.get(job_id)
            if proc:
                if os.name == 'nt':
                    proc.terminate()
                else:
                    os.killpg(proc.pid, signal.SIGTERM)
                threading.Thread(target=self._finish_stop, args=(proc,), daemon=True).start()
            return dict(ok=True)

    @staticmethod
    def _finish_stop(proc):
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                proc.kill()
            else:
                os.killpg(proc.pid, signal.SIGKILL)

    def status(self):
        with self.lock:
            return dict(ok=True, jobs=copy.deepcopy(self.jobs[:20]), demo=self.demo,
                        root=str(self.root), python=self.python)

    def checkpoints(self):
        result = []
        log_dir = self.root / 'logs/rsl_rl'
        if log_dir.is_dir():
            for p in log_dir.glob('**/model_*.pt'):
                result.append(dict(path=str(p.relative_to(self.root)), size=p.stat().st_size,
                                   modified=p.stat().st_mtime))
                if len(result) >= 500:
                    break
        return sorted(result, key=lambda x: x['modified'], reverse=True)
