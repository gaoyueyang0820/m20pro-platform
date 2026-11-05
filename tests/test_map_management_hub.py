"""Desktop activation/cleanup ordering with synthetic demo maps only."""
import json,shutil,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20_platform import Platform
from m20.platform_core import Store


class MapManagementHubTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.app=Platform(Store(Path(self.tmp.name)/'workspace.json'));self.hub=self.app.hub
        maps=self.hub.demo_maps.maps;shutil.copytree(maps/'demo-yard',maps/'candidate')
        (maps/'candidate/.m20-project.json').write_text(json.dumps(dict(sessions=[dict(map='candidate',base_map='demo-yard',includes_complete_base=True,created=1)])))
    def tearDown(self):self.hub.close();self.tmp.cleanup()
    def wait(self,reply):
        for _ in range(200):
            result=self.hub.operations.get(reply['job_id'])
            if result['status'] in ['completed','failed']:return result
            time.sleep(.01)
        self.fail('job did not finish')
    def test_confirmed_apply_syncs_before_archiving_previous_map(self):
        result=self.wait(self.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['result']['cleanup']['ok']);self.assertFalse((self.hub.demo_maps.maps/'demo-yard').exists())
        self.assertEqual(self.hub.demo_maps.library()['active'],'candidate')
    def test_sync_failure_retains_old_map(self):
        with patch.object(self.hub,'sync_map',side_effect=ValueError('synthetic sync failure')):
            result=self.wait(self.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'failed');self.assertTrue((self.hub.demo_maps.maps/'demo-yard').is_dir())
    def test_ordinary_apply_does_not_clean_old_map(self):
        result=self.wait(self.hub.post('/api/maps/library/apply',dict(name='candidate')))
        self.assertEqual(result['status'],'completed',result);self.assertTrue((self.hub.demo_maps.maps/'demo-yard').is_dir())

    def test_restart_does_not_recreate_archived_demo_map(self):
        result=self.wait(self.hub.post('/api/maps/library/apply',dict(name='candidate',cleanup_previous=True,cleanup_base='demo-yard')))
        self.assertEqual(result['status'],'completed',result)
        restored=Platform(Store(Path(self.tmp.name)/'workspace.json'))
        try:self.assertNotIn('demo-yard',[m['name'] for m in restored.hub.demo_maps.library()['maps']])
        finally:restored.hub.close()


if __name__=='__main__':unittest.main()
