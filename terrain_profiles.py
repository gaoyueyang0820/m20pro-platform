"""Documented M20 terrain/gait recommendations, without robot I/O.

This is an advisory layer. A gait recommendation is not proof of traversability
and does not widen the currently validated perception/control envelope.
Sources are the project's 2026-10-23 vendor archive; see the companion note.
"""
import copy
import math


GAITS = {
    0x1001: dict(name='标准基础', family='standard', role='manual_structured'),
    0x1003: dict(name='标准楼梯', family='standard', role='manual_steps_slopes'),
    0x3002: dict(name='敏捷平地', family='agile', role='autonomous_flat_grass_slope'),
    0x3003: dict(name='敏捷楼梯', family='agile', role='steps_slopes'),
}
LEGACY_GAITS = {0x1002: '高台：仅作历史/设备反馈展示，未列入当前四种公开轴控步态'}
USE_MODES = {0: '常规', 1: '导航', 2: '辅助'}
OOA_STATES = {0: '未启动', 1: '空闲', 2: '无障碍物', 3: '避障介入'}

PROFILES = {
    'indoor_flat': dict(label='室内平地', gait=0x3002,
                       perception=['fresh_supported_floor', 'obstacle_and_drop_detection']),
    'narrow': dict(label='狭窄通道', gait=0x3002,
                   perception=['oriented_body_clearance', 'swept_turn_clearance',
                               'headroom', 'fresh_supported_floor']),
    'slope': dict(label='连续坡地', gait=0x3002,
                  perception=['surface_slope_and_cross_slope', 'support_continuity',
                              'traction_and_slip', 'negative_obstacles']),
    'stairs': dict(label='台阶/楼梯', gait=0x3003,
                   perception=['step_height_depth_and_width', 'landing_space',
                               'support_continuity', 'negative_obstacles']),
    'soft_ground': dict(label='松软地面/沙地', gait=None,
                        perception=['sinkage', 'slip', 'support_strength',
                                    'wheel_leg_progress', 'negative_obstacles']),
    'rough_ground': dict(label='崎岖山地', gait=None,
                         perception=['local_3d_terrain', 'support_continuity',
                                     'step_geometry', 'traction_and_slip',
                                     'negative_obstacles']),
}


def _finite(value, name, minimum=0):
    if isinstance(value, bool):raise ValueError(name+' must be a finite number')
    try:value=float(value)
    except (TypeError, ValueError):raise ValueError(name+' must be a finite number')
    if not math.isfinite(value) or value<minimum:raise ValueError(name+' is out of range')
    return value


def recommend_profile(terrain, *, control='axis', prefer_assisted=False,
                      native_assist_ready=False, ceiling_m=None,
                      slope_deg=None, step_height_m=None, validated_gait=None):
    """Return configuration advice, not an actuator command or a go decision.

    control: axis (2/21), native_auto (NavMode=1), native_direct (NavMode=0).
    validated_gait records an independently field-validated configuration for
    soft/rough ground; an environment name alone cannot invent a vendor gait.
    """
    if terrain not in PROFILES:raise ValueError('Unknown terrain profile')
    if control not in ['axis', 'native_auto', 'native_direct']:raise ValueError('Unknown control path')
    if validated_gait is not None and (isinstance(validated_gait,bool) or validated_gait not in GAITS):
        raise ValueError('Only the four documented daily gaits are eligible')
    profile=copy.deepcopy(PROFILES[terrain]);gait=profile['gait'];notes=[];compatible=True
    if gait is None:
        gait=validated_gait
        notes.append('没有公开的独立沙漠/山地步态编码；按地形几何和现场验证选择已有步态')
    limits={}
    geometry=None
    if terrain in ['slope', 'stairs']:
        limits['documented_navigation_slope_deg']=30
        if terrain=='stairs':limits['documented_navigation_step_height_m']=.25
        measurements=[]
        if slope_deg is not None:
            slope=_finite(slope_deg,'slope_deg',-90)
            if abs(slope)>90:raise ValueError('slope_deg is out of range')
            measurements.append(abs(slope)<=30)
        if terrain=='stairs' and step_height_m is not None:measurements.append(_finite(step_height_m,'step_height_m')<=.25)
        if False in measurements:geometry=False
        elif len(measurements)==(2 if terrain=='stairs' else 1):geometry=True
        if geometry is False:notes.append('已超出公开导航步态的对应地形范围，不能套用本建议')
        notes.append('须接入相应三维地形判断；现有平地地面拟合不代表坡地/楼梯可通行')
    if terrain in ['soft_ground','rough_ground']:
        notes.append('已有配置通过现场验证也不等于当前二维探索已能判断陷足、打滑和落差')
    if control=='axis':
        mode=2 if prefer_assisted else 0
        if prefer_assisted:
            if not native_assist_ready:
                mode=None;compatible=False;notes.append('辅助感知链尚未就绪，不自动切换为辅助模式')
            if ceiling_m is not None and _finite(ceiling_m,'ceiling_m')<1:
                mode=None;compatible=False;notes.append('厂商提示低于1米顶高会使辅助模式误判；需要有顶高判断的通行策略')
        nav_mode=nav_speed=obs_mode=None
    else:
        mode=1;nav_mode=1 if control=='native_auto' else 0;nav_speed=1;obs_mode=0
        if gait not in [0x3002,0x3003]:
            compatible=False;notes.append('原厂导航路径仅公开敏捷平地/敏捷楼梯两种步态')
        if control=='native_auto' and (gait!=0x3002 or terrain in ['stairs','soft_ground','rough_ground']):
            compatible=False;notes.append('原厂自主导航仅支持敏捷平地＋低速；复杂台阶不能只改步态后继续NavMode=1')
        if terrain=='stairs':notes.append('楼梯应单独验证分段直线/台阶感知，本建议不会自动关闭停避障')
    return dict(terrain=terrain,label=profile['label'],recommended_gait=gait,
                recommended_gait_name=GAITS[gait]['name'] if gait else None,
                use_mode=mode,control_path=control,control_path_compatible=compatible,
                nav_mode=nav_mode,nav_speed=nav_speed,obs_mode=obs_mode,
                geometry_within_documented_range=geometry,documented_limits=limits,
                required_perception=profile['perception'],notes=notes,
                recommendation_only=True,path_is_proven=False,
                current_level_floor_pipeline_sufficient=terrain=='indoor_flat')


def transition_step(basic, motion, *, desired_mode, desired_gait,
                    basic_age=0., motion_age=0., pending=False,
                    navigation_active=False, sdk_active=False):
    """One proposed transition step. The caller executes and waits for readback.

    No command should be sent again while pending=True. After fresh matching
    feedback for that step, the caller clears pending and asks for the next.
    Standstill tolerances below are console engineering settings, not vendor
    terrain limits; they do not change any movement-envelope checks.
    """
    if isinstance(desired_mode,bool) or desired_mode not in USE_MODES:raise ValueError('Unknown usage mode')
    if isinstance(desired_gait,bool) or desired_gait not in GAITS:raise ValueError('Unsupported daily gait')
    if desired_mode==1 and desired_gait not in [0x3002,0x3003]:raise ValueError('Gait is not a documented navigation gait')
    row=dict(action='wait',command=None,ready=False)
    if sdk_active or navigation_active:return dict(row,reason='先结束当前SDK输出或导航任务')
    if _finite(basic_age,'basic_age')>1.5 or _finite(motion_age,'motion_age')>.5:
        return dict(row,reason='等候新鲜状态反馈')
    if basic.get('HES')!=0 or basic.get('Sleep') or basic.get('MotionState')!=17:
        return dict(row,reason='等候已唤醒且MotionState=17的日常运控状态')
    for field,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)]:
        value=motion.get(field)
        if value is None or isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            return dict(row,reason='等候有效的实测运动状态')
        if abs(value)>limit:return dict(row,action='stop_before_switch',reason='先停稳，再切换模式/步态')
    if pending:return dict(row,reason='等前一条切换指令的新鲜状态确认，不重复发送')
    if basic.get('ControlUsageMode')!=desired_mode:
        return dict(row,action='set_mode',reason='先切使用模式；这一步会重置基础步态',
                    command=dict(type=1101,command=5,items=dict(Mode=desired_mode)),
                    expected_feedback=dict(ControlUsageMode=desired_mode))
    if basic.get('Gait')!=desired_gait:
        return dict(row,action='set_gait',reason='使用模式已确认，再切目标步态',
                    command=dict(type=2,command=23,items=dict(GaitParam=desired_gait)),
                    expected_feedback=dict(Gait=desired_gait))
    return dict(action='ready',command=None,ready=True,reason='模式/步态读回一致；路径与地形仍由通行判断负责')
