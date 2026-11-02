"""打招呼能力单元测试：纯逻辑，不发任何网络包。运行：python -m pytest tests/test_greeting.py -q"""
import json
import struct

import greeting as g


def parse_frame(buf):
    assert buf[:4] == g.SYNC
    length, seq = struct.unpack_from('<HH', buf, 4)
    assert length == len(buf) - 16
    assert buf[8] == 1
    obj = json.loads(buf[16:16 + length].decode('utf-8'))
    return obj['PatrolDevice']


def test_light_on_frame_structure():
    buf = g.frame(g.LIGHT_TYPE, g.LIGHT_CMD, g.light_on_items('breathe'), seq=7)
    pd = parse_frame(buf)
    assert pd['Type'] == 1101 and pd['Command'] == 14
    items = pd['Items']
    assert items['CustomMode'] is True
    assert len(items['Led']) == 2                    # Led[0] 头灯 Led[1] 尾灯
    led_type, color, cycle = g.LIGHT_STYLES['breathe']
    assert items['Led'][0] == {'Type': led_type, 'Color': [color], 'Cycle': cycle}
    assert items['Led'][1] == items['Led'][0]        # 缺省尾灯与头灯同款


def test_light_styles_are_valid_and_off_resets():
    for style in g.LIGHT_STYLES:
        items = g.light_on_items(style)
        assert items['Led'][0]['Type'] in (0, 1, 2, 4, 5)
        assert items['Led'][0]['Color'][0] in (0, 1, 2, 3)
    off = g.light_off_items()
    assert off == {'CustomMode': False}


def test_light_cycle_must_be_positive_int():
    try:
        g._led_entry(4, 2, 0)
        raise AssertionError('Cycle=0 应被拒绝')
    except ValueError:
        pass


def test_nod_plan_shape_and_bounds():
    frames = g.nod_plan()
    assert len(frames) == int(20 * 2 * g.NOD_PERIOD_S)   # 20Hz × 2 次 × 单周期
    amplitudes = [f['Pitch'] for f in frames]
    assert max(amplitudes) <= g.NOD_AMPLITUDE + 1e-9
    assert min(amplitudes) >= -g.NOD_AMPLITUDE - 1e-9
    assert max(amplitudes) >= g.NOD_AMPLITUDE - .05      # 幅度确实用满，不被无意缩小
    assert abs(amplitudes[0]) < .01                      # 从零开始平滑
    for f in frames:                                     # 全程不产生位移/转向
        assert f['X'] == f['Y'] == f['Z'] == f['Roll'] == f['Yaw'] == 0.0
    pd = parse_frame(g.frame(g.AXIS_TYPE, g.AXIS_CMD, frames[0]))
    assert (pd['Type'], pd['Command']) == (2, 21)


def test_bob_plan_uses_z_axis_only():
    frames = g.bob_plan(periods=1)
    assert len(frames) == int(20 * g.BOB_PERIOD_S)
    for f in frames:
        assert f['X'] == f['Y'] == f['Pitch'] == f['Roll'] == f['Yaw'] == 0.0
    peak = max(abs(f['Z']) for f in frames)
    assert peak >= g.BOB_AMPLITUDE - .05


def test_locomotion_snapshot_and_restore_order():
    """打招呼会切换运动模式：必须记录原步态并在结束后恢复。"""
    snap = g.locomotion_snapshot({'BasicStatus': {'Gait': 12290, 'ControlUsageMode': 0}})
    assert snap == {'gait': 12290, 'mode': 0}
    cmds = g.restore_commands(snap)
    assert [c[1] for c in cmds] == [5, 23]           # 先使用模式、后步态
    assert cmds[0][2] == {'Mode': 0}
    assert cmds[1][2] == {'GaitParam': 12290}


def test_locomotion_snapshot_ignores_unknown_values():
    snap = g.locomotion_snapshot({'BasicStatus': {'Gait': 12345, 'ControlUsageMode': 9}})
    assert snap == {'gait': None, 'mode': None}
    assert g.restore_commands(snap) == []
    assert g.restore_commands(None) == []


def test_zero_plan_all_zero():
    frames = g.zero_plan(0.2)
    assert len(frames) == 4                          # 20Hz × 0.2s
    for f in frames:
        assert all(v == 0.0 for v in f.values())


def test_amplitude_guards():
    for bad in (0., .8):          # 0 无效；超过 0.7 的硬上限拒绝
        try:
            g.nod_plan(amplitude=bad)
            raise AssertionError('幅度 %s 应被拒绝' % bad)
        except ValueError:
            pass
    for bad in (0., .7):          # 起伏硬上限 0.6
        try:
            g.bob_plan(amplitude=bad)
            raise AssertionError('起伏幅度 %s 应被拒绝' % bad)
        except ValueError:
            pass


def test_precheck_stand_ready():
    ok, _ = g.precheck({'BasicStatus': {'MotionState': 1, 'HES': 0, 'Sleep': False,
                                        'ControlUsageMode': 0}},
                       {'MotionStatus': {'LinearX': 0., 'LinearY': 0., 'OmegaZ': 0.}})
    assert ok


def test_precheck_accepts_rl_control_state():
    """MotionState=17（RL 控制）是完成站立后的正常可遥控状态，必须允许。"""
    ok, message = g.precheck({'BasicStatus': {'MotionState': 17, 'HES': 0, 'Sleep': False,
                                              'ControlUsageMode': 0}},
                             {'MotionStatus': {'LinearX': 0., 'LinearY': 0., 'OmegaZ': 0.}})
    assert ok, message


def test_precheck_rejects_sit_estop_idle_and_moving():
    cases = [
        ({'MotionState': 4, 'HES': 0}, None, '趴下'),
        ({'MotionState': 2, 'HES': 0}, None, '急停'),
        ({'MotionState': 0, 'HES': 0}, None, '可遥控'),
        ({'MotionState': 3, 'HES': 0}, None, '可遥控'),
        ({'MotionState': 17, 'HES': 3}, None, 'HES'),
        ({'MotionState': 17, 'HES': 0}, {'MotionStatus': {'LinearX': .3}}, '运动'),
        ({'MotionState': 17, 'HES': 0, 'ControlUsageMode': 1}, None, '模式'),
    ]
    for basic, motion, keyword in cases:
        ok, message = g.precheck({'BasicStatus': basic}, motion)
        assert not ok, (basic, message)
        assert keyword in message, (keyword, message)


def test_precheck_missing_state():
    ok, message = g.precheck(None)
    assert not ok and '基础状态' in message
