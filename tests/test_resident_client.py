"""Desktop delegation/reconnect tests; only synthetic RPC and local fixtures."""
import base64
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import Mock,patch
from m20.collection_store import CollectionJournal, robot_digest
from m20.platform_core import RobotLink
from m20.resident_navigation_client import ResidentNavigationClient


class ResidentClientTests(unittest.TestCase):
    def setUp(self):
        self.acl=patch('m20.platform_host.private_file',lambda path:None);self.acl.start();self.addCleanup(self.acl.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.raw=b'P5\n3 3\n255\n'+bytes([254])*9
        (self.root/'maps').mkdir();(self.root/'maps/map.pgm').write_bytes(self.raw)
        self.map=dict(id='map',robot_map_name='synthetic',grid_id=0,resolution=.1,origin=[0.,0.,0.])
        connections=Obj(value=dict(aos_identity_alias='synthetic-robot',targets={'nos':{'enabled':True}}),endpoint=lambda _:('http://synthetic.invalid','synthetic-token'))
        self.app=Obj(store=Obj(path=self.root/'workspace.json'),lock=threading.RLock(),hub=Obj(connections=connections),demo=False,
            maps=lambda:[self.map],_record_route_completion=Mock(),nav=dict(status='idle'),link=None)
        self.c=ResidentNavigationClient(self.app);self.c.tick=Mock()
        self.c.collection_sync.autostart=False
        self.addCleanup(self.c.close)
        self.nav=dict(status='running',disconnect_policy='finish',options=dict(NavMode=1,Gait=0x3002,Speed=1,ObsMode=0,Manner=0),route=dict(map_id='map',points=[dict(x=.15,y=.15,yaw=0,type=1)]))
        self.c.prepared(self.nav);self.app.nav=self.nav;self.c.wake=Mock()
    def grid(self):return dict(robot_map_name='synthetic',pgm=base64.b64encode(self.raw).decode(),resolution=.1,origin=[0.,0.,0.],yaml_sha256='b'*64)
    def test_only_complete_route_is_uploaded_after_capability_and_hash_checks(self):
        calls=[]
        def rpc(path,body=None):
            calls.append((path,body))
            if path=='/missions/capabilities':return dict(protocol=1,available=True,policies=['finish'])
            if path=='/maps/grid':return self.grid()
            return dict(ok=True)
        self.c._request=rpc;self.c._submit()
        self.assertEqual([p for p,_ in calls],['/missions/capabilities','/maps/grid','/missions/start'])
        self.assertEqual(calls[-1][1]['points'],self.nav['route']['points'])
        self.assertEqual(calls[-1][1]['disconnect_policy'],'finish')
    def test_capability_missing_or_changed_map_never_uploads_task(self):
        for kind in ['unavailable','policy','map']:
            self.c.ref['submitted']=False;calls=[]
            def rpc(path,body=None):
                calls.append(path)
                if path=='/missions/capabilities':
                    return dict(protocol=1,available=kind!='unavailable',policies=['stop'] if kind=='policy' else ['finish'])
                return dict(self.grid(),origin=[1,1,0])
            self.c._request=rpc
            with self.assertRaises(ValueError):self.c._submit()
            self.assertNotIn('/missions/start',calls);self.assertFalse(self.c.ref['submitted'])
    def test_cancel_during_map_read_wins_before_upload(self):
        calls=[]
        def rpc(path,body=None):
            calls.append(path)
            if path=='/missions/capabilities':return dict(protocol=1,available=True,policies=['finish'])
            self.c.cancel();return self.grid()
        self.c._request=rpc;self.c._submit();self.assertNotIn('/missions/start',calls)
    def test_gos_without_map_odometry_reports_reason_before_submitting(self):
        self.c.ref.update(executor='gos',policy='stop')
        def rpc(path,body=None):
            if path=='/missions/capabilities':
                return dict(protocol=1,available=True,executor='gos')
            if path=='/sensors/status':return dict(odom=None,lidar_publishers=2)
            self.fail('unexpected request: '+path)
        self.c._request=rpc;self.c._run()
        self.assertEqual(self.nav['status'],'failed')
        self.assertIn('/ODOM',self.nav['message'])
        self.assertFalse(self.c.ref['submitted'])
    def test_restart_reconciles_only_and_never_replays_an_uncertain_start(self):
        self.c.ref['submitted']=False;self.c._save()
        restored=ResidentNavigationClient(self.app);restored.wake=Mock();calls=[]
        def rpc(path,body=None):
            calls.append(path)
            return dict(mission=dict(id=restored.ref['id'],phase='completed',completed=1,stop_confirmed=True))
        restored._request=rpc;restored._run()
        self.assertNotIn('/missions/start',calls);self.assertEqual(restored.nav['status'],'completed')
    def test_cancel_receipt_for_not_started_request_is_final_without_motion(self):
        self.c.cancel();self.c._request=Mock(return_value=dict(matched=False,revoked=True,mission=dict(phase='idle')))
        self.c._run();self.assertFalse(self.c.busy());self.assertEqual(self.nav['status'],'cancelled')
    def test_sent_stop_or_failed_link_is_never_treated_as_confirmed_parking(self):
        self.c.cancel();self.c._observe(dict(mission=dict(id=self.c.ref['id'],phase='stopping',completed=0,stop_confirmed=False)))
        self.assertTrue(self.c.busy());self.assertEqual(self.nav['status'],'cancelling')
    def test_changed_robot_identity_prevents_old_rpc_and_pulses(self):
        self.app.hub.connections.value['aos_identity_alias']='another-robot'
        self.c._request=Mock();self.c._run();self.c._request.assert_not_called()
    def test_closing_observer_link_does_not_send_zero_or_cancel(self):
        link=RobotLink.__new__(RobotLink);link.set_velocity=Mock();link._send_axis=Mock();link.sock=Mock();link._stop=False
        link.close(stop=False);link.set_velocity.assert_not_called();link._send_axis.assert_not_called();self.assertTrue(link._stop)

    def collecting_gos(self):
        robot=robot_digest('synthetic-robot')
        self.app.hub.connections.value['targets']['gos']=dict(enabled=True)
        self.app.store.value=dict(history=[])
        self.c.ref.update(executor='gos',policy='stop',submitted=True,collection_robot=robot)
        self.nav.update(executor='gos',disconnect_policy='stop',
                        collection_run=dict(id=self.c.ref['id'],robot=robot,executor='gos',sync_protocol=1))
        return self.nav['collection_run']

    def observation(self,completed=0,high=0,terminal=False):
        return dict(mission=dict(id=self.c.ref['id'],phase='completed' if terminal else 'running',
                    stage='completed' if terminal else 'tracking',completed=completed,
                    stop_confirmed=terminal,collection=[dict(spec=dict(kind='people_count'),samples=high)],
                    collection_journal=dict(collection_sync_protocol=1,robot=self.c.ref['collection_robot'],
                                            high_watermark=high,complete=terminal)))

    def test_waypoint_reconnect_and_terminal_queue_once_not_every_heartbeat(self):
        run=self.collecting_gos()
        self.c.collection_sync.notify=Mock()
        self.c._observe(self.observation(high=1))
        self.c._observe(self.observation(high=2))
        self.c._observe(self.observation(high=3))
        self.assertEqual(self.c.collection_sync.notify.call_count,1)
        self.c._observe(self.observation(completed=1,high=3))
        self.assertEqual(self.c.collection_sync.notify.call_count,2)
        self.c._collection_link_lost=True
        self.c._observe(self.observation(completed=1,high=4))
        self.assertEqual(self.c.collection_sync.notify.call_count,3)
        self.c._observe(self.observation(completed=1,high=4,terminal=True))
        self.c.collection_sync.notify.assert_called_with(run,final=True,target=4)
        self.assertFalse(self.c.busy())
        self.assertEqual(self.nav['status'],'completed')

    def test_failed_station_only_run_keeps_history_reference_before_data_arrives(self):
        self.collecting_gos()
        response=self.observation(terminal=True,high=1)
        response['mission'].update(phase='failed',stage='failed',collection=[])
        self.c._observe(response)
        self.assertEqual(self.nav['status'],'failed')
        self.app._record_route_completion.assert_called_once_with(self.nav)
        self.assertTrue(self.nav['collection_run'])
        self.assertTrue(self.c.collection_sync.incoming)

    def test_terminal_client_tick_still_starts_cache_reconciliation(self):
        self.collecting_gos()
        self.c.ref['terminal']=True
        self.nav['status']='completed'
        self.c.collection_sync.start=Mock()
        ResidentNavigationClient.tick(self.c)
        self.c.collection_sync.start.assert_called_once()
        self.assertTrue(self.c.collection_sync.incoming)
        self.assertIsNone(self.c.worker)

    def test_sync_thread_failure_cannot_fail_navigation_observation(self):
        self.collecting_gos()
        self.c.collection_sync.autostart=True
        with patch('m20.collection_sync.threading.Thread.start',side_effect=RuntimeError('synthetic thread limit')):
            self.c._observe(self.observation(high=1))
        self.assertEqual(self.nav['status'],'running')
        self.assertTrue(self.c.busy())
        self.assertIn('补传线程',self.c.collection_sync.error)
        self.assertIsNone(self.c.collection_sync.worker)

    def test_slow_collection_rpc_does_not_hold_app_lock_or_observation_thread(self):
        run=self.collecting_gos()
        remote=CollectionJournal(self.root/'synthetic-robot-data')
        remote.ensure_run(run['id'],robot=run['robot'])
        blocked,release,finished=threading.Event(),threading.Event(),threading.Event()
        self.addCleanup(release.set)
        def rpc(base,path,body,timeout,token):
            if path=='/missions/capabilities':return dict(collection_sync_protocol=1)
            self.assertEqual(path,'/missions/collection')
            blocked.set();release.wait(3)
            return remote.page(run['id'],cursor=body['cursor'],robot=run['robot'])
        def observe():
            self.c._observe(self.observation(high=2));finished.set()
        with patch('m20.resident_navigation_client.rpc',side_effect=rpc):
            self.c.collection_sync.autostart=True
            self.c._observe(self.observation(high=1))
            self.assertTrue(blocked.wait(1))
            worker=threading.Thread(target=observe)
            worker.start()
            self.assertTrue(finished.wait(1),'navigation observation was blocked by cache transport')
            worker.join(1)
            self.assertEqual(self.nav['status'],'running')
            release.set()
            self.c.collection_sync.close()

    def test_cached_history_remains_readable_offline_without_live_rpc(self):
        run=self.collecting_gos()
        remote=CollectionJournal(self.root/'synthetic-robot-data')
        remote.append(run['id'],dict(kind='people_count',sampled_at=1.,values=dict(people=3)),robot=run['robot'])
        remote.finish(run['id'],robot=run['robot'])
        cache=self.c.collection_sync.cache
        cache.schedule(run['id'],robot=run['robot'],final=True)
        cache.ingest(run['id'],remote.page(run['id'],cursor=0,robot=run['robot']),robot=run['robot'])
        self.app.hub.connections.value['aos_identity_alias']='different-robot'
        with patch('m20.resident_navigation_client.rpc',side_effect=AssertionError('offline history must be local')):
            result=self.c.collection_page(run['id'],0)
        self.assertEqual(result['rows'][0]['values']['people'],3)
        self.assertTrue(result['stale']);self.assertFalse(result['live'])
        self.assertTrue(result['complete'])

    def test_cached_station_results_are_deduplicated_and_never_relabelled_live(self):
        run=self.collecting_gos()
        row=dict(seq=1,kind='station_result',index=0,people=2,sampled_at=10.)
        state=dict(protocol=1,cursor=1,high_watermark=1,complete=False,pending=False,state='synced')
        self.c._collection_updated(run,state,[row,row])
        self.assertEqual(len(self.nav['station_results']),1)
        self.assertEqual(self.nav['station_results'][0]['sampled_at'],10.)
        self.assertTrue(self.nav['station_results'][0]['stale'])
        self.assertFalse(self.nav['station_results'][0]['fresh'])
        self.c._observe(self.observation(high=1))
        self.assertEqual(len(self.nav['station_results']),1)

    def test_legacy_manual_page_keeps_offset_but_does_not_claim_durable_sync(self):
        run=self.collecting_gos();run['sync_protocol']=0
        with patch('m20.resident_navigation_client.rpc',return_value=dict(ok=True,run_id=run['id'],rows=[dict(sampled_at=2.)],next_offset=None)) as rpc:
            result=self.c.collection_page(run['id'],0)
        self.assertEqual(rpc.call_args.args[2],dict(id=run['id'],offset=0))
        self.assertEqual(result['source'],'legacy_remote')
        self.assertFalse(result['sync_supported']);self.assertFalse(result['cached'])
        self.assertTrue(result['stale']);self.assertFalse(result['complete'])

    def test_typed_unavailable_page_survives_real_rpc_error_handling(self):
        run=self.collecting_gos()
        def response(request,timeout):
            if request.full_url.endswith('/missions/capabilities'):
                return io.StringIO(json.dumps(dict(ok=True,collection_sync_protocol=1)))
            body=json.loads(request.data)
            self.assertTrue(body['response_envelope'])
            return io.StringIO(json.dumps(dict(ok=True,page=dict(ok=False,available=False,
                collection_sync_protocol=0,error='旧任务仅支持offset',run_id=run['id'],rows=[]))))
        with patch('m20.platform_core.urlopen',side_effect=response):
            self.c.collection_sync.notify(run,final=True)
            self.c.collection_sync.run_once()
        state=self.c.collection_sync.snapshot(run)
        self.assertFalse(state['supported'])
        self.assertEqual(state['state'],'unsupported')
        self.assertFalse(state['pending'])
        self.assertEqual(state['cursor'],0)

    def test_station_only_gos_binds_robot_and_does_not_inherit_nos_heartbeat_limit(self):
        self.c.ref['terminal']=True
        self.app.hub.connections.value['targets']['gos']=dict(enabled=True)
        self.nav['disconnect_policy']='stop'
        self.nav['route']['points'][0]['station_action']=dict(camera='front',pan='none')
        self.nav['route']['options']=dict(disconnect_timeout_s=25,max_duration_s=45)
        self.c.prepared(self.nav,executor='gos')
        self.assertEqual(self.nav['collection_run']['robot'],robot_digest('synthetic-robot'))
        calls=[]
        def rpc(path,body=None):
            calls.append((path,body))
            if path=='/missions/capabilities':
                return dict(protocol=1,available=True,executor='gos',collection_sync_protocol=1)
            if path=='/sensors/status':return dict(odom={})
            if path=='/maps/grid':return dict(self.grid(),active=True)
            return dict(ok=True)
        self.c._request=rpc;self.c._submit()
        body=next(b for p,b in calls if p=='/missions/start')
        self.assertEqual(body['collection_robot'],robot_digest('synthetic-robot'))
        self.assertNotIn('disconnect_timeout_s',body)
        self.assertNotIn('max_duration_s',body)

    def test_nos_limits_use_explicit_nav_override_and_keep_collection_gate(self):
        self.nav['route']['options']=dict(disconnect_timeout_s=20,max_duration_s=300)
        self.nav['disconnect_timeout_s']=9
        calls=[]
        def rpc(path,body=None):
            calls.append((path,body))
            if path=='/missions/capabilities':return dict(protocol=1,available=True,policies=['finish'])
            if path=='/maps/grid':return self.grid()
            return dict(ok=True)
        self.c._request=rpc;self.c._submit()
        body=next(b for p,b in calls if p=='/missions/start')
        self.assertEqual((body['disconnect_timeout_s'],body['max_duration_s']),(9,300))
        self.c.ref['terminal']=True
        self.nav['route']['options']['collectors']=[dict(kind='people_count',camera='front')]
        with self.assertRaisesRegex(ValueError,'暂不支持'):
            self.c.prepared(self.nav)
        self.nav['route']['options'].pop('collectors')
        self.nav['disconnect_timeout_s']=31
        with self.assertRaisesRegex(ValueError,'超出范围'):
            self.c.prepared(self.nav)


if __name__=='__main__':unittest.main()
