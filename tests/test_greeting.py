"""Pure protocol/feedback regression tests, discovered by unittest and CI."""
import copy
import json
import math
import struct
import unittest

import greeting as g


def basic(**kwargs):
    return {'BasicStatus': dict(MotionState=17, HES=0, Sleep=False,
                               ControlUsageMode=0, Gait=0x3002, **kwargs)}


def motion(**kwargs):
    result = {'LinearX': 0., 'LinearY': 0., 'OmegaZ': 0.}
    result.update(kwargs)
    return {'MotionStatus': result}


def record(items, stamp=10.):return dict(items=items, monotonic_at=stamp)


class GreetingTests(unittest.TestCase):
    def test_frame_and_light_protocol(self):
        data = g.frame(1101, 14, g.light_on_items(), seq=7)
        self.assertEqual(data[:4], g.SYNC)
        self.assertEqual(struct.unpack_from('<HH', data, 4), (len(data)-16, 7))
        self.assertEqual(data[8], 1)
        pd = json.loads(data[16:])['PatrolDevice']
        self.assertEqual((pd['Type'], pd['Command']), (1101, 14))
        self.assertEqual(pd['Items']['Led'], [{'Type': 4, 'Color': [2], 'Cycle': 1}]*2)
        for style in g.LIGHT_STYLES:self.assertEqual(len(g.light_on_items(style)['Led']), 2)
        self.assertEqual(g.light_off_items(), {'CustomMode': False})

    def test_light_entries_independent_and_strict(self):
        items = g.light_on_items()
        items['Led'][0]['Color'][0] = 3
        self.assertEqual(items['Led'][1]['Color'], [2])
        for cycle in (0, 1.5, True, float('nan'), float('inf'), 61):
            with self.subTest(cycle=cycle), self.assertRaises(ValueError):g._led_entry(4, 2, cycle)
        for style in ('bad', [], None):
            with self.assertRaises(ValueError):g.light_on_items(style)

    def test_waveforms_request_no_xy_or_yaw(self):
        # Requested zeros do not assert physical immobility.
        for make, axis, amplitude, expected in [(g.nod_plan, 'Pitch', .5, 64), (g.bob_plan, 'Z', .45, 72)]:
            plan = make()
            self.assertEqual(len(plan), expected)
            self.assertAlmostEqual(plan[0][axis], 0.)
            self.assertLessEqual(max(abs(f[axis]) for f in plan), amplitude)
            self.assertGreater(max(abs(f[axis]) for f in plan), amplitude-.05)
            for f in plan:self.assertTrue(all(v == 0 for k, v in f.items() if k != axis))
        self.assertEqual(len(g.zero_plan(.2)), 4)
        self.assertTrue(all(not any(f.values()) for f in g.zero_plan()))

    def test_frame_and_plan_parameter_boundaries(self):
        for kwargs in ({'amplitude': float('nan')}, {'amplitude': float('inf')}, {'amplitude': True},
                       {'amplitude': 0}, {'amplitude': .8}, {'periods': 1.5}, {'periods': True},
                       {'periods': 5}, {'period_s': 0}, {'period_s': float('nan')}, {'hz': 0}, {'hz': 21}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):g.nod_plan(**kwargs)
        with self.assertRaises(ValueError):g.bob_plan(amplitude=.7)
        with self.assertRaises(ValueError):g.frame(2, 21, {'X': float('nan')})
        with self.assertRaises(ValueError):g.frame(True, 21, {})
        with self.assertRaises(ValueError):g.frame(2, 21, {}, seq=65536)
        with self.assertRaises(ValueError):g.zero_plan(float('inf'))

    def test_ready_records_and_explicit_age(self):
        self.assertTrue(g.precheck(record(basic()), record(motion()), now=10.1)[0])
        self.assertTrue(g.precheck(basic(), motion(), basic_age=.1, motion_age=.1)[0])
        self.assertFalse(g.precheck(basic(), motion())[0])

    def test_transition_is_not_standing_authorization(self):
        b = basic();b['BasicStatus']['MotionState'] = 1
        ok, reason = g.precheck(record(b), record(motion()), now=10.1)
        self.assertFalse(ok)
        self.assertIn('过渡', reason)
        for state in (0, 2, 3, 4, 6, 17., True):
            b['BasicStatus']['MotionState'] = state
            self.assertFalse(g.precheck(record(b), record(motion()), now=10.1)[0])

    def test_missing_and_invalid_basic_fields_fail_closed(self):
        for field in ('HES', 'Sleep', 'MotionState', 'ControlUsageMode', 'Gait'):
            b = basic();del b['BasicStatus'][field]
            self.assertFalse(g.precheck(record(b), record(motion()), now=10.1)[0], field)
        for field, values in [('HES', [True, 0., None, 3]), ('Sleep', [None, 0., True]),
                              ('ControlUsageMode', [1, 9, 0., False]), ('Gait', [1, 0x3002+0., True])]:
            for value in values:
                b = basic();b['BasicStatus'][field] = value
                self.assertFalse(g.precheck(record(b), record(motion()), now=10.1)[0], (field,value))

    def test_missing_nonfinite_and_moving_feedback_rejected(self):
        for field in ('LinearX', 'LinearY', 'OmegaZ'):
            for value in (None, True, float('nan'), float('inf'), '0', .3, -.3):
                m = motion();m['MotionStatus'][field] = value
                self.assertFalse(g.precheck(record(basic()), record(m), now=10.1)[0], (field,value))
            m = motion();del m['MotionStatus'][field]
            self.assertFalse(g.precheck(record(basic()), record(m), now=10.1)[0])

    def test_feedback_independent_expiry_future_and_nan(self):
        self.assertFalse(g.precheck(record(basic(), 7), record(motion()), now=10.1)[0])
        self.assertFalse(g.precheck(record(basic()), record(motion(), 9), now=10.1)[0])
        for stamp in (float('nan'), float('inf'), True, '10', 11):
            self.assertFalse(g.precheck(record(basic(), stamp), record(motion()), now=10.1)[0])
        for age in (-1, float('nan'), float('inf'), True):
            self.assertFalse(g.precheck(basic(), motion(), basic_age=age, motion_age=0)[0])

    def test_malformed_records_rejected_without_exception(self):
        for b in (None, [], {}, {'items': []}, {'items': {'BasicStatus': []}}):
            self.assertFalse(g.precheck(b, record(motion()), now=10.1)[0])
            self.assertEqual(g.locomotion_snapshot(b), dict(mode=None, gait=None))

    def test_snapshot_and_no_implicit_restore(self):
        snap = g.locomotion_snapshot(record(basic()))
        self.assertEqual(snap, {'gait': 0x3002, 'mode': 0})
        self.assertEqual(g.restore_commands(snap), [])
        self.assertEqual(g.restore_commands(snap, snap), [])
        self.assertEqual(g.restore_commands(None), [])

    def test_restore_requires_explicit_still_matching_owned_change(self):
        before = {'mode': 0, 'gait': 0x3002}
        owned = {'mode': 2, 'gait': 0x1001}
        cmds = g.restore_commands(before, dict(owned), owned_changes=owned)
        self.assertEqual([(c[0],c[1],c[2]) for c in cmds], [(1101,5,{'Mode':0}), (2,23,{'GaitParam':0x3002})])
        self.assertEqual(g.restore_commands(before, {'mode':1, 'gait':0x1001}, owned_changes=owned), [])
        self.assertEqual(g.restore_commands(before, {'mode':2, 'gait':0x1003}, owned_changes=owned), [])
        self.assertEqual(g.restore_commands(before, dict(owned), owned_changes={'mode':2}), [])
        self.assertEqual(g.restore_commands({'mode':0., 'gait':0x3002}, owned, owned_changes=owned), [])
        # Restoring mode resets gait to basic; do not send a redundant gait command.
        cmds = g.restore_commands({'mode':0,'gait':0x1001}, owned, owned_changes=owned)
        self.assertEqual([c[1] for c in cmds], [5])


if __name__ == '__main__':unittest.main()
