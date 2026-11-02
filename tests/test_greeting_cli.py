"""CLI lifecycle through fake transport/clock; never creates a real socket."""
import copy
import io
import threading
import unittest
from contextlib import redirect_stdout, redirect_stderr
from types import SimpleNamespace
from unittest.mock import patch

import greeting as g
import m20_greet as cli


class Clock:
    def __init__(self):self.now = 100.
    def monotonic(self):return self.now
    def sleep(self, seconds):self.now += max(0., seconds)


class FakeLink:
    def __init__(self, clock):
        self.clock = clock
        self.error = None
        self.axes_sent = self.light_sent = False
        self.closed = False
        self.commands = []
        self.basic = dict(MotionState=17, HES=0, Sleep=False, ControlUsageMode=0, Gait=0x3002, Direction=0)
        self.motion = dict(LinearX=0., LinearY=0., OmegaZ=0.)
        self.motion_stamp = None
        self.on_command = None
        self.on_record = None
    def command(self, type_, cmd, items):
        self.commands.append((type_, cmd, copy.deepcopy(items)))
        if self.on_command:self.on_command(type_, cmd, items)
    def record(self, type_, cmd):
        if self.on_record:self.on_record(type_, cmd)
        items = {'BasicStatus':copy.deepcopy(self.basic)} if cmd == 6 else {'MotionStatus':copy.deepcopy(self.motion)}
        stamp = self.motion_stamp if cmd == 4 and self.motion_stamp is not None else self.clock.now
        return dict(items=items, monotonic_at=stamp)
    def close(self):self.closed = True


class CLITests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.link = FakeLink(self.clock)
        self.patches = [patch.object(cli.time, 'monotonic', self.clock.monotonic),
                        patch.object(cli.time, 'sleep', self.clock.sleep),
                        patch.object(cli, 'Link', return_value=self.link)]
        for p in self.patches:p.start()
    def tearDown(self):
        for p in reversed(self.patches):p.stop()
    def run_main(self, *args):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return cli.main(['--seconds', '.05', *args])
    def axes(self):return [c for c in self.link.commands if c[:2] == (2,21)]
    def assert_no_mode_or_gait(self):
        self.assertFalse([c for c in self.link.commands if c[:2] in [(1101,5),(2,23)]])

    def test_light_only_works_while_prone_and_never_sends_axes(self):
        self.link.basic['MotionState'] = 4
        self.assertEqual(self.run_main(), 0)
        self.assertEqual(self.axes(), [])
        self.assert_no_mode_or_gait()
        self.assertTrue(self.link.closed)
        self.assertEqual(self.link.commands[-1], (1101,14,{'CustomMode':False}))

    def test_status_sends_no_mutations_even_with_motion_flags(self):
        self.assertEqual(self.run_main('--status', '--nod', '--stand'), 0)
        self.assertEqual(self.link.commands, [])
        self.assertTrue(self.link.closed)

    def test_missing_motion_feedback_blocks_before_lights_or_axes(self):
        del self.link.motion['OmegaZ']
        self.assertEqual(self.run_main('--nod'), 1)
        self.assertEqual(self.link.commands, [])
        self.assertTrue(self.link.closed)

    def test_normal_gesture_waits_for_new_stopped_feedback_no_restore(self):
        self.assertEqual(self.run_main('--nod'), 0)
        self.assertTrue(any(c[2]['Pitch'] != 0 for c in self.axes()))
        self.assertFalse(any(self.axes()[-1][2].values()))
        self.assert_no_mode_or_gait()
        self.assertFalse(self.link.axes_sent)

    def test_mode_change_during_light_prevents_first_axis(self):
        def command(t,c,items):
            if (t,c) == (1101,14) and items['CustomMode']:self.link.basic['ControlUsageMode'] = 2
        self.link.on_command = command
        self.assertEqual(self.run_main('--nod'), 1)
        self.assertEqual(self.axes(), [])
        self.assert_no_mode_or_gait()
        self.assertEqual(self.link.basic['ControlUsageMode'], 2)

    def test_operator_gait_change_during_gesture_stops_without_restoring(self):
        def command(t,c,items):
            if (t,c) == (2,21) and items['Pitch']:self.link.basic['Gait'] = 0x1003
        self.link.on_command = command
        self.assertEqual(self.run_main('--nod'), 1)
        self.assertEqual(self.link.basic['Gait'], 0x1003)
        self.assert_no_mode_or_gait()
        self.assertFalse(any(self.axes()[-1][2].values()))
        self.assertEqual(sum(bool(c[2]['Pitch']) for c in self.axes()), 1)

    def test_ctrl_c_has_nonzero_exit_and_owned_cleanup_only(self):
        def command(t,c,items):
            if (t,c) == (2,21) and items['Pitch']:raise KeyboardInterrupt()
        self.link.on_command = command
        self.assertEqual(self.run_main('--nod'), 130)
        self.assertFalse(any(self.axes()[-1][2].values()))
        self.assert_no_mode_or_gait()
        self.assertTrue(self.link.closed)

    def test_cleanup_error_does_not_hide_original_ctrl_c(self):
        def command(t,c,items):
            if (t,c) == (2,21):
                if items['Pitch']:raise KeyboardInterrupt()
                if len(self.axes()) > 1:raise OSError('stop transport unavailable')
        self.link.on_command = command
        self.assertEqual(self.run_main('--nod'), 130)
        self.assertTrue(self.link.closed)

    def test_motion_ages_independently_during_light(self):
        self.link.motion_stamp = self.clock.now-.49
        self.assertEqual(self.run_main('--nod'), 1)
        self.assertEqual(self.axes(), [])

    def test_stand_transition_never_authorizes_axis_early(self):
        self.link.basic['MotionState'] = 1
        began = self.clock.now
        def receive(t,c):
            if self.clock.now >= began+.2:self.link.basic['MotionState'] = 17
        self.link.on_record = receive
        ok, _ = cli.ensure_stand(self.link, SimpleNamespace(stand=False))
        self.assertTrue(ok)
        self.assertGreaterEqual(self.clock.now, began+.2)
        self.assertEqual(self.link.commands, [])

    def test_explicit_stand_does_not_change_usage_mode_or_gait(self):
        self.link.basic['MotionState'] = 4
        def command(t,c,items):
            if (t,c) == (2,22):self.link.basic['MotionState'] = 1
        self.link.on_command = command
        began = self.clock.now
        def receive(t,c):
            if self.clock.now > began+.2:self.link.basic['MotionState'] = 17
        self.link.on_record = receive
        ok, _ = cli.ensure_stand(self.link, SimpleNamespace(stand=True))
        self.assertTrue(ok)
        self.assertEqual(self.link.commands, [(2,22,{'MotionParam':1})])

    def test_unsafe_stand_never_sends_request(self):
        self.link.basic.update(MotionState=4,HES=3)
        ok, _ = cli.ensure_stand(self.link, SimpleNamespace(stand=True))
        self.assertFalse(ok)
        self.assertEqual(self.link.commands, [])

    def test_sender_lag_cancels_instead_of_bursting_frames(self):
        def sleep(seconds):self.clock.now += max(0., seconds)+.2
        with patch.object(cli.time, 'sleep', sleep), self.assertRaisesRegex(RuntimeError,'节拍'):
            cli.run_frames(self.link, g.nod_plan(), cli._signature(self.link.record(1002,6)))
        self.assertEqual(self.axes(), [])

    def test_bad_cli_arguments_reject_before_transport(self):
        for args in (['--seconds','nan'], ['--seconds','inf'], ['--seconds','-1'],
                     ['--port','0'], ['--port','65536'], ['--ip','not-an-ip']):
            with self.subTest(args=args), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                cli.main(args)
            self.assertEqual(raised.exception.code, 2)
        cli.Link.assert_not_called()


class ReceiveTests(unittest.TestCase):
    def bare_link(self):
        link = object.__new__(cli.Link)
        link.ip = '192.0.2.7'
        link.lock = threading.Lock()
        link.snapshots = {}
        return link
    def test_packet_validation_source_ip_and_type_command_key(self):
        link = self.bare_link()
        good = g.frame(1002,6,{'BasicStatus':{'MotionState':17}})
        link._accept(good, ('192.0.2.8',30000))
        self.assertEqual(link.record(1002,6), {})
        link._accept(good, ('192.0.2.7',30000))
        first = link.record(1002,6)
        link._accept(g.frame(100,6,{'unrelated':True}), ('192.0.2.7',30000))
        self.assertEqual(link.record(1002,6), first)
        self.assertEqual(link.record(100,6)['items'], {'unrelated':True})
        first['items']['BasicStatus']['MotionState'] = 2
        self.assertEqual(link.record(1002,6)['items']['BasicStatus']['MotionState'],17)
        for broken in (good[:-1], good+b'x', good[:8]+b'\x02'+good[9:], b'x'*16):
            self.assertIsNone(cli.Link._parse(broken))

    def test_heartbeat_does_not_freshen_old_status(self):
        link = self.bare_link()
        with patch.object(cli.time,'monotonic',return_value=10.):
            link._accept(g.frame(1002,6,{'BasicStatus':{'MotionState':17}}), ('192.0.2.7',30000))
        with patch.object(cli.time,'monotonic',return_value=20.):
            link._accept(g.frame(100,100,{}), ('192.0.2.7',30000))
            self.assertEqual(g.feedback(link.record(1002,6))[1],10.)


if __name__ == '__main__':unittest.main()
