"""Desktop/GOS lease contracts. All RPCs are synthetic."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from m20.onboard_exploration import OnboardExploration


class OnboardExplorationTests(unittest.TestCase):
    def setUp(self):
        self.task=OnboardExploration(SimpleNamespace(done=False),'a'*32,'b'*32,'fixture',('synthetic','token'))

    def test_heartbeat_carries_capture_and_returns_only_after_confirmed_stop(self):
        replies=iter([{'mission':{'phase':'running'}},
                      {'mission':{'phase':'completed','stop_confirmed':True}}])
        status=dict(phase='collecting',mapping=dict(request_id='b'*32))
        seen=[]
        with patch.object(self.task,'_call',side_effect=lambda *_args,**_kw:next(replies)) as rpc:
            result=self.task.run({},lambda:status,seen.append,lambda:True)
        self.assertEqual(result['phase'],'completed')
        self.assertTrue(self.task.stopped.is_set())
        self.assertEqual([args.args[0] for args in rpc.call_args_list],['/missions/exploration/start','/missions/pulse'])
        self.assertEqual(rpc.call_args.args[1]['capture_id'],'b'*32)

    def test_unconfirmed_start_is_cancelled_and_never_sent_again(self):
        replies=[ValueError('lost reply'),{'revoked':True,'matched':False,'mission':{}}]
        with patch.object(self.task,'_call',side_effect=replies) as rpc:
            with self.assertRaisesRegex(ValueError,'lost reply'):
                self.task.run({},lambda:{},lambda _:None,lambda:True)
        self.assertTrue(self.task.stopped.is_set())
        self.assertEqual([args.args[0] for args in rpc.call_args_list],['/missions/exploration/start','/missions/cancel'])

    def test_capture_change_stops_without_renewing_the_onboard_lease(self):
        replies=[{'mission':{'phase':'running'}},{'mission':{'stop_confirmed':True}}]
        with patch.object(self.task,'_call',side_effect=replies) as rpc:
            with self.assertRaisesRegex(RuntimeError,'采集已结束或已变化'):
                self.task.run({},lambda:dict(phase='collecting',mapping=dict(request_id='c'*32)),lambda _:None,lambda:True)
        self.assertTrue(self.task.stopped.is_set())
        self.assertEqual([args.args[0] for args in rpc.call_args_list],['/missions/exploration/start','/missions/cancel'])

    def test_unavailable_status_never_blocks_the_heartbeat(self):
        # A NOS read that times out is not evidence the capture ended. The old
        # ordering raised before pulse and silently stopped renewing the lease.
        replies=iter([{'mission':{'phase':'running'}},
                      {'mission':{'phase':'completed','stop_confirmed':True}}])
        def unavailable():raise ValueError('NOS 地图状态读取超时')
        with patch.object(self.task,'_call',side_effect=lambda *_args,**_kw:next(replies)) as rpc:
            result=self.task.run({},unavailable,lambda _:None,lambda:True)
        self.assertEqual(result['phase'],'completed')
        self.assertEqual([args.args[0] for args in rpc.call_args_list],['/missions/exploration/start','/missions/pulse'])
        self.assertEqual(self.task.heartbeat['sent'],1)
        self.assertEqual(self.task.heartbeat['stale'],1)
        self.assertEqual(self.task.heartbeat['status_reads'],1)
        self.assertEqual(result['heartbeat']['sent'],1)

    def test_heartbeat_reports_its_own_cadence(self):
        replies=iter([{'mission':{'phase':'running'}},
                      {'mission':{'stop_confirmed':True}}])
        status=dict(phase='collecting',mapping=dict(request_id='b'*32))
        with patch.object(self.task,'_call',side_effect=lambda *_args,**_kw:next(replies)):
            result=self.task.run({},lambda:status,lambda _:None,lambda:True)
        beat=result['heartbeat']
        self.assertEqual(beat['sent'],1)
        self.assertIsNotNone(beat['last_sent'])
        self.assertEqual(beat['failed'],0)
        self.assertEqual(beat['status_failures'],0)
