"""Map edits keep source connection, navigation caches and confirmed cleanup."""
import copy
import json
import shutil
import threading
import time
import unittest
from unittest.mock import patch
import test_platform as cases
from m20.localization_runtime import OperationCancelled


class MapManagementReviewTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def cached(self,name='candidate',id='cache',ip=None,demo=True):
        row=dict(id=id,name=name,robot_map_name=name,robot_ip=ip,demo=demo,verified_for_robot=True)
        self.app.store.value['maps'].append(row);return row
    def candidate(self):
        maps=self.app.hub.demo_maps.maps
        shutil.copytree(maps/'demo-yard',maps/'candidate')
        (maps/'candidate/.m20-project.json').write_text(json.dumps(dict(sessions=[dict(map='candidate',base_map='demo-yard',includes_complete_base=True,created=1)])))
        return maps
    def done(self,reply):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            row=self.app.hub.operations.get(reply['job_id'])
            if row['status'] in ['completed','failed']:return row
            time.sleep(.005)
        self.fail('operation did not complete')
    def test_late_grid_from_previous_connection_is_not_imported(self):
        before=copy.deepcopy(self.app.store.value['maps'])
        real=self.app.hub.remote_maps
        def switched(*args,**kwargs):
            grid=real(*args,**kwargs);self.app.hub._context_generation+=1;return grid
        with patch.object(self.app.hub,'remote_maps',side_effect=switched):
            with self.assertRaises(OperationCancelled):self.app.hub.sync_map(dict(name='demo-yard',grid_id=0))
        self.assertEqual(self.app.store.value['maps'],before)
    def test_wrong_grid_or_map_response_never_becomes_verified_cache(self):
        real=self.app.hub.remote_maps;grid=real('/maps/grid',dict(name='demo-yard',grid_id=0))
        for changed in [dict(grid,grid_id=7),dict(grid,robot_map_name='other-map')]:
            with patch.object(self.app.hub,'remote_maps',return_value=changed):
                with self.assertRaisesRegex(ValueError,'不一致'):self.app.hub.sync_map(dict(name='demo-yard',grid_id=0))
        self.assertEqual(self.app.store.value['maps'],[])
    def test_old_rename_result_cannot_rename_new_connection_cache(self):
        self.candidate();row=self.cached();entered=threading.Event();release=threading.Event();real=self.app.hub.remote_maps
        def delayed(path,*args,**kwargs):
            result=real(path,*args,**kwargs)
            if path=='/maps/rename':entered.set();release.wait(2)
            return result
        with patch.object(self.app.hub,'remote_maps',side_effect=delayed):
            reply=self.app.hub.post('/api/maps/library/rename',dict(name='candidate',display_name='old robot edit'))
            self.assertTrue(entered.wait(1));self.app.hub._context_generation+=1;row['name']='new robot cache';release.set()
            result=self.done(reply)
        self.assertEqual(result['status'],'failed');self.assertIn('连接已变化',result['error']);self.assertEqual(row['name'],'new robot cache')
    def test_live_rename_only_updates_matching_robot_cache(self):
        a=self.app;a.demo=False;a.link=cases.FakeLink();a.link.ip='192.168.88.2'
        current=self.cached(ip=a.link.ip,demo=False);other=self.cached(id='other',ip='192.168.88.3',demo=False)
        def remote(path,req=None,**kwargs):
            if path=='/maps/build/status':return dict(in_progress=False)
            if path=='/maps/rename':return dict(ok=True,name='candidate',display_name='renamed')
            raise AssertionError(path)
        with patch.object(a.hub,'remote_maps',side_effect=remote):result=self.done(a.hub.post('/api/maps/library/rename',dict(name='candidate',display_name='renamed')))
        self.assertEqual(result['status'],'completed');self.assertEqual(current['name'],'renamed');self.assertEqual(other['name'],'candidate')
    def test_delete_hides_navigation_cache_and_restore_keeps_identity_unverified(self):
        self.candidate();row=self.cached()
        self.app.store.value['routes']=[dict(id='route',map_id=row['id'],name='saved route',points=[dict(x=0,y=0)])]
        deleted=self.done(self.app.hub.post('/api/maps/library/delete',dict(name='candidate')))
        self.assertEqual(deleted['status'],'completed');self.assertTrue(row['archived']);self.assertNotIn(row,self.app.maps())
        restored=self.done(self.app.hub.post('/api/maps/library/restore',dict(id=deleted['result']['trash_id'])))
        self.assertEqual(restored['status'],'completed');self.assertFalse(row['archived']);self.assertFalse(row['verified_for_robot'])
        self.assertEqual(row['id'],'cache');self.assertEqual(self.app.store.value['routes'][0]['map_id'],'cache')
    def test_discard_cleanup_only_archives_matching_hidden_observation(self):
        maps=self.candidate();service=self.app.hub.demo_maps
        capture='synthetic-capture'
        service.demo_build.update(phase='discarded',observation_ownership='unique',
                                  temporary_observations=[dict(name='candidate',ownership_verified=True)])
        service.demo_stop['expected_start_request_id']=capture
        def cleanup(name='candidate',identity=capture):
            return self.done(self.app.hub.post('/api/maps/library/delete',dict(name=name,observation_capture=identity)))
        self.assertIn('文件已变化',cleanup()['error'])
        (maps/'candidate/.m20-session-source.json').write_text(json.dumps(dict(discarded=True,capture_id=capture)))
        self.assertIn('归属已变化',cleanup(identity='another-capture')['error'])
        self.assertIn('归属已变化',cleanup(name='demo-yard')['error'])
        self.assertEqual(service.dispatch('/maps/trash',{})['maps'],[])
        result=cleanup()
        self.assertEqual(result['status'],'completed',result)
        self.assertFalse((maps/'candidate').exists());self.assertEqual(service.library()['active'],'demo-yard')
        self.assertEqual([row['name'] for row in service.dispatch('/maps/trash',{})['maps']],['candidate'])
    def test_unconfirmed_or_different_cleanup_base_only_applies_without_archiving(self):
        for confirmed in [None,'other-map']:
            with self.subTest(confirmed=confirmed):
                # Separate activation is needed for each pass, just like an operator.
                if not (self.app.hub.demo_maps.maps/'candidate').exists():self.candidate()
                self.app.hub.demo_maps.dispatch('/maps/apply',dict(name='demo-yard'))
                result=self.done(self.app.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base=confirmed)))
                self.assertEqual(result['status'],'completed',result);self.assertTrue(result['result']['cleanup']['retained'])
                self.assertEqual(self.app.hub.demo_maps.library()['active'],'candidate');self.assertTrue((self.app.hub.demo_maps.maps/'demo-yard').is_dir())
    def test_referenced_base_is_retained_after_candidate_activation(self):
        maps=self.candidate();self.cached('demo-yard')
        self.app.store.value['routes']=[dict(id='route',map_id='cache',name='inspection',points=[dict(x=0,y=0)])]
        result=self.done(self.app.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'completed');self.assertTrue(result['result']['cleanup']['retained'])
        self.assertIn('路线',result['result']['cleanup']['message']);self.assertTrue((maps/'demo-yard').exists())
    def test_confirmed_permanent_delete_keeps_saved_route_without_extra_block(self):
        self.candidate();row=self.cached();deleted=self.done(self.app.hub.post('/api/maps/library/delete',dict(name='candidate')))
        self.app.store.value['routes']=[dict(id='route',map_id=row['id'],name='inspection',points=[dict(x=0,y=0)])]
        result=self.done(self.app.hub.post('/api/maps/library/purge',dict(id=deleted['result']['trash_id'],confirmed_name='candidate')))
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.app.hub.demo_maps.dispatch('/maps/trash',{})['maps'],[])
        self.assertEqual(self.app.store.value['routes'][0]['map_id'],row['id'])
        self.assertNotIn(row,self.app.maps())
    def test_rename_during_navigation_and_map_capture_does_not_stop_tasks(self):
        self.candidate();self.app.mapping=True;self.app.nav['status']='running'
        self.app.velocity=[.4,0,0];self.app.sdk['armed']=True;self.app.hub.busy='other-job'
        self.app.hub.demo_maps.demo_build['in_progress']=True
        result=self.done(self.app.hub.post('/api/maps/library/rename',dict(name='candidate',display_name='Updated name')))
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.app.nav['status'],'running');self.assertEqual(self.app.velocity,[.4,0,0])
        self.assertTrue(self.app.mapping);self.assertEqual(self.app.hub.busy,'other-job')
    def test_healthy_sdk_service_is_reused_without_ssh_or_restart(self):
        self.app.hub.connections.value['targets']['sdk'].update(enabled=True,deployed=True)
        with patch.object(self.app.hub,'proxy',return_value=dict(ok=True)) as proxy,patch.object(self.app.hub.connections,'start') as start,patch.object(self.app.hub.connections,'service') as service:
            self.assertTrue(self.app.hub.ensure_sdk_service(lambda *args:None)['ok'])
        proxy.assert_called_once_with('sdk','/status',timeout=2);start.assert_not_called();service.assert_not_called()
    def test_demo_seed_is_not_resurrected_in_navigation_after_cleanup(self):
        self.candidate()
        result=self.done(self.app.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'completed',result)
        self.assertNotIn('demo',[m['id'] for m in self.app.maps()]);self.assertIn('candidate',[m['robot_map_name'] for m in self.app.maps()])
    def test_connection_change_after_actual_archive_does_not_claim_old_map_retained(self):
        maps=self.candidate();real=self.app.hub.remote_maps
        def switched(path,*args,**kwargs):
            result=real(path,*args,**kwargs)
            if path=='/maps/cleanup-previous':self.app.hub._context_generation+=1
            return result
        with patch.object(self.app.hub,'remote_maps',side_effect=switched):
            result=self.done(self.app.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'failed');self.assertIn('已完成旧图归档',result['error']);self.assertFalse((maps/'demo-yard').exists())


if __name__=='__main__':unittest.main()
