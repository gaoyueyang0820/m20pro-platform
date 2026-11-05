"""Greeting protocol helpers; no network, process, or actuator I/O.

Custom lights are documented. Pitch/Z gestures remain experimental: the official
M20 moving-gait interface documents only X/Y/Yaw as effective. Zero requested
translation is not proof of zero physical displacement or unchanged posture.
"""
import copy
import json
import math
import struct
import time

SYNC=b'\xEB\x91\xEB\x90'
ASDU_FORMAT_JSON=1
AXIS_TYPE,AXIS_CMD=2,21
LIGHT_TYPE,LIGHT_CMD=1101,14
STATE_IDLE,STATE_STAND,STATE_ESTOP,STATE_BOOT_DAMP,STATE_SIT,STATE_RL=0,1,2,3,4,17
LIGHT_STYLES={'breathe':(4,2,1),'flow':(2,3,1),'blink':(5,1,1),'solid':(0,2,1)}
AXIS_HZ=20
ZERO_HOLD_S=.4  # Repeated zero requests; neither an ACK nor proof of a physical stop.
NOD_AMPLITUDE=.5
NOD_PERIOD_S=1.6
BOB_AMPLITUDE=.45
BOB_PERIOD_S=1.8
GAITS=(0x1001,0x1003,0x3002,0x3003)


def _number(value,name,low,high):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError('%s 必须是 [%s,%s] 内的有限数值'%(name,low,high))
    return float(value)


def _integer(value,name,low,high):
    if isinstance(value,bool) or not isinstance(value,int) or not low<=value<=high:
        raise ValueError('%s 必须是 [%s,%s] 内的整数'%(name,low,high))
    return value


def frame(type_,command,items,seq=0,at=None):
    _integer(type_,'Type',0,65535);_integer(command,'Command',0,65535);_integer(seq,'seq',0,65535)
    if not isinstance(items,dict):raise ValueError('Items 必须是对象')
    body=json.dumps({'PatrolDevice':{'Type':type_,'Command':command,'Time':at or time.strftime('%Y-%m-%d %H:%M:%S'),'Items':items}},separators=(',',':'),allow_nan=False).encode('utf-8')
    if len(body)>65535:raise ValueError('ASDU 过长')
    return SYNC+struct.pack('<HH',len(body),seq)+bytes([ASDU_FORMAT_JSON])+b'\0'*7+body


def _led_entry(led_type,color,cycle):
    _integer(led_type,'灯光 Type',0,5);_integer(color,'灯光 Color',0,3);_integer(cycle,'Cycle',1,60)
    if led_type not in (0,1,2,4,5):raise ValueError('灯光 Type 只能是 0/1/2/4/5')
    return {'Type':led_type,'Color':[color],'Cycle':cycle}


def light_on_items(style='breathe',tail=None):
    if not isinstance(style,str) or style not in LIGHT_STYLES:raise ValueError('未知灯语预设')
    head=_led_entry(*LIGHT_STYLES[style])
    if tail is not None and (not isinstance(tail,(list,tuple)) or len(tail)!=3):raise ValueError('尾灯配置需要 Type/Color/Cycle')
    return {'CustomMode':True,'Led':[head,copy.deepcopy(head) if tail is None else _led_entry(*tail)]}


def light_off_items():return {'CustomMode':False}


def _axis_items(x=0.,y=0.,z=0.,roll=0.,pitch=0.,yaw=0.):
    return {k:round(_number(v,k,-1,1),3) for k,v in dict(X=x,Y=y,Z=z,Roll=roll,Pitch=pitch,Yaw=yaw).items()}


def _plan(axis,amplitude,maximum,periods,period_s,hz):
    amplitude=_number(amplitude,'幅度',.001,maximum);period_s=_number(period_s,'单周期秒数',.5,5.)
    _integer(periods,'周期次数',1,4);_integer(hz,'采样频率',AXIS_HZ,AXIS_HZ)
    return [_axis_items(**{axis:amplitude*math.sin(2*math.pi*(k/hz)/period_s)}) for k in range(math.ceil(hz*periods*period_s))]


def nod_plan(amplitude=NOD_AMPLITUDE,periods=2,period_s=NOD_PERIOD_S,hz=AXIS_HZ):
    """Pitch-only requested axes, experimental; not a no-displacement promise."""
    return _plan('pitch',amplitude,.7,periods,period_s,hz)


def bob_plan(amplitude=BOB_AMPLITUDE,periods=2,period_s=BOB_PERIOD_S,hz=AXIS_HZ):
    return _plan('z',amplitude,.6,periods,period_s,hz)


def zero_plan(seconds=ZERO_HOLD_S,hz=AXIS_HZ):
    seconds=_number(seconds,'零轴时长',.001,2.);_integer(hz,'采样频率',AXIS_HZ,AXIS_HZ)
    return [_axis_items() for _ in range(math.ceil(seconds*hz))]


def feedback(packet,age=None,now=None):
    """Return Items and receive age; raw Items require an explicit trusted age.

    Full records prefer monotonic_at. Future/non-finite timestamps are invalid;
    unrelated heartbeat traffic cannot make an old state record fresh.
    """
    if not isinstance(packet,dict):return {},math.inf
    items=packet.get('items',packet)
    if not isinstance(items,dict):return {},math.inf
    if age is None:
        if 'monotonic_at' in packet:
            stamp=packet['monotonic_at'];clock=time.monotonic() if now is None else now
        elif 'at' in packet:stamp=packet['at'];clock=time.time()
        else:return items,math.inf
        if isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp):return items,math.inf
        age=clock-stamp
    if isinstance(age,bool) or not isinstance(age,(int,float)) or not math.isfinite(age) or age<0:return items,math.inf
    return items,float(age)


def _basic(packet):
    if not isinstance(packet,dict):return {}
    items=packet.get('items',packet)
    status=items.get('BasicStatus',items) if isinstance(items,dict) else {}
    return status if isinstance(status,dict) else {}


def locomotion_snapshot(basic):
    """Observed mode/gait only; does not attribute changes to this operation."""
    status=_basic(basic);gait=status.get('Gait');mode=status.get('ControlUsageMode')
    return dict(gait=gait if isinstance(gait,int) and not isinstance(gait,bool) and gait in GAITS else None,
                mode=mode if isinstance(mode,int) and not isinstance(mode,bool) and mode in (0,1,2) else None)


def restore_commands(snapshot,current=None,*,owned_changes=None):
    """Restore only explicitly owned, still-matching changes, never guessed ones.

    Caller must additionally hold operation ownership and require fresh stopped
    feedback before EACH send/readback. owned_changes is the complete expected
    current {'mode':..., 'gait':...} state written by that operation. A differing
    field means an operator/other source may have intervened: return no commands.
    Greetings do not themselves change mode/gait, so their default is no restore.
    """
    if not all(isinstance(v,dict) for v in [snapshot,current,owned_changes]):return []
    if set(owned_changes)!={'mode','gait'}:return []
    current=current if 'mode' in current and 'gait' in current else locomotion_snapshot(current)
    if any(current.get(k)!=owned_changes[k] for k in ['mode','gait']):return []
    if any(isinstance(v,bool) or not isinstance(v,int) for d in [snapshot,current,owned_changes] for v in [d.get('mode'),d.get('gait')]):return []
    if any(d['mode'] not in (0,1,2) or d['gait'] not in GAITS for d in [current,owned_changes]):return []
    mode,gait=snapshot.get('mode'),snapshot.get('gait')
    if mode not in (0,1,2) or gait not in GAITS:return []
    out=[]
    if current.get('mode')!=mode:
        out.append((1101,5,{'Mode':mode},'恢复本操作拥有的使用模式'))
        # Documented mode switching resets gait to basic; gait follows readback.
        if gait!=0x1001:out.append((2,23,{'GaitParam':gait},'恢复本操作拥有的步态'))
    elif current.get('gait')!=gait:out.append((2,23,{'GaitParam':gait},'恢复本操作拥有的步态'))
    return out


def precheck(basic,motion=None,*,basic_age=None,motion_age=None,now=None):
    """Require current RL(17), known control flags and finite stopped feedback.

    State 1 is a standing transition, not authorization for sustained axes.
    This is not a substitute for caller-owned navigation/SDK/task arbitration.
    """
    b,ba=feedback(basic,basic_age,now);m,ma=feedback(motion,motion_age,now)
    if not b or ba>2.:return False,'未收到新鲜基础状态（1002/6）'
    b=b.get('BasicStatus',b)
    if not isinstance(b,dict):return False,'基础状态格式无效'
    if type(b.get('HES')) is not int or b['HES']!=0:return False,'HES 未明确为 0，禁止动作类问候'
    if type(b.get('Sleep')) not in (int,bool) or b['Sleep'] not in (0,False):return False,'休眠状态未知或尚未唤醒'
    state=b.get('MotionState')
    if type(state) is not int or state!=STATE_RL:
        return False,'等待起立过渡完成并读回 MotionState=17' if state==STATE_STAND else '机器人未处于可遥控 RL 状态（MotionState=%s）'%state
    mode=b.get('ControlUsageMode')
    if type(mode) is not int or mode not in (0,2):return False,'使用模式未知或不允许轴指令，请确认常规/辅助模式'
    gait=b.get('Gait')
    if type(gait) is not int or gait not in GAITS:return False,'步态反馈未知，不能执行实验性姿态轴'
    if not m or ma>.5:return False,'未收到新鲜运动反馈（1002/4）'
    m=m.get('MotionStatus',m)
    if not isinstance(m,dict):return False,'运动反馈格式无效'
    for key,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)]:
        value=m.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):return False,'运动反馈 %s 缺失或无效'%key
        if abs(value)>=limit:return False,'机器人仍在运动，请停稳后再打招呼'
    return True,'已收到新鲜的可遥控与静止反馈；姿态轴效果仍需实机验证'
