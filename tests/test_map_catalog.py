"""Synthetic map folders only. No robot, network, or operational map files."""
import json
from pathlib import Path
import tempfile
import subprocess
import unittest
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_catalog import MapCatalog
from bridge.map_service import MapService


class MapCatalogTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.maps=self.root/'maps';self.maps.mkdir()
        self.make('old');self.make('new');(self.maps/'active').symlink_to('old',target_is_directory=True)
        self.catalog=MapCatalog(self.maps)
    def tearDown(self):self.temp.cleanup()
    def make(self,name):
        folder=self.maps/name;folder.mkdir();(folder/'full_cloud.pcd').write_text('synthetic point cloud')
        (folder/'occ_grid.pgm').write_bytes(b'P5\n1 1\n255\n\xff');(folder/'occ_grid.yaml').write_text('resolution: 1\norigin: [0,0,0]\n');return folder
    def activate(self,name):
        (self.maps/'active').unlink();(self.maps/'active').symlink_to(name,target_is_directory=True)
    def merged(self):
        (self.maps/'new/.m20-project.json').write_text(json.dumps(dict(sessions=[dict(map='new',base_map='old',includes_complete_base=True,created=1)])))
    def test_rename_preserves_active_identity_and_files(self):
        self.catalog.execute('rename',dict(name='old',display_name='一楼大厅'))
        service=MapService(self.maps,self.root/'data',True);row=next(m for m in service.library()['maps'] if m['name']=='old')
        self.assertEqual(row['display_name'],'一楼大厅');self.assertTrue(row['active'])
        self.assertEqual((self.maps/'old/full_cloud.pcd').read_text(),'synthetic point cloud')
    def test_delete_restore_is_lossless_and_never_activates(self):
        deleted=self.catalog.execute('delete',dict(name='new'));self.assertFalse((self.maps/'new').exists())
        self.assertEqual(len(self.catalog.trash_list()),1)
        self.catalog.execute('restore',dict(id=deleted['trash_id']))
        self.assertEqual((self.maps/'new/full_cloud.pcd').read_text(),'synthetic point cloud')
        self.assertEqual((self.maps/'active').resolve().name,'old');self.assertEqual(self.catalog.trash_list(),[])
    def test_active_busy_and_unsafe_paths_are_rejected(self):
        for name in ['old','active','../outside']:
            with self.subTest(name=name),self.assertRaises(ValueError):self.catalog.execute('delete',dict(name=name))
        busy=MapCatalog(self.maps,lambda:(_ for _ in ()).throw(ValueError('busy')))
        with self.assertRaisesRegex(ValueError,'busy'):busy.execute('delete',dict(name='new'))
        self.assertTrue((self.maps/'new').exists())
    def test_cleanup_requires_complete_candidate_and_successful_activation(self):
        self.merged()
        with self.assertRaises(ValueError):self.catalog.execute('cleanup-previous',dict(name='new',base='old'))
        self.activate('new');result=self.catalog.execute('cleanup-previous',dict(name='new',base='old'))
        self.assertTrue(result['recoverable']);self.assertEqual((self.maps/'active').resolve().name,'new')
        self.assertEqual(self.catalog.trash_list()[0]['reason'],'merged-into:new')
    def test_incomplete_or_unrelated_candidate_retains_old_map(self):
        self.activate('new')
        with self.assertRaises(ValueError):self.catalog.execute('cleanup-previous',dict(name='new',base='old'))
        self.merged();(self.maps/'new/occ_grid.pgm').unlink()
        with self.assertRaisesRegex(ValueError,'缺少'):self.catalog.execute('cleanup-previous',dict(name='new',base='old'))
        self.assertTrue((self.maps/'old').is_dir())
    def test_cross_map_reference_and_restore_collision_do_not_destroy_data(self):
        (self.maps/'old/reference').symlink_to('../new/full_cloud.pcd')
        with self.assertRaisesRegex(ValueError,'引用'):self.catalog.execute('delete',dict(name='new'))
        (self.maps/'old/reference').unlink();deleted=self.catalog.execute('delete',dict(name='new'));self.make('new')
        with self.assertRaisesRegex(ValueError,'未覆盖'):self.catalog.execute('restore',dict(id=deleted['trash_id']))
        self.assertEqual(len(self.catalog.trash_list()),1)
    def test_demo_mapping_blocks_delete(self):
        service=MapService(self.maps,self.root/'data',True);service.demo_build['in_progress']=True
        with self.assertRaisesRegex(ValueError,'采集'):service.dispatch('/maps/delete',dict(name='new'))

    def test_finished_normal_mapping_does_not_permanently_block_catalog(self):
        from bridge import root_actions
        self.make('scan-output')
        records={'mapping':dict(state='command_finished',started=1,name='scan'),
                 'stop_mapping':dict(state='command_finished',started=2)}
        with patch('bridge.mapping_job.read_job',side_effect=lambda kind:records[kind]),patch('bridge.mapping_job.unowned_processes',return_value=[]):
            result=root_actions.execute(dict(action='catalog-delete',name='new'),self.maps,self.root/'data')
        self.assertTrue(result['recoverable'])

    def test_helper_business_error_is_not_reported_as_permission_failure(self):
        service=MapService(self.maps,self.root/'data')
        result=subprocess.CompletedProcess([],1,json.dumps(dict(ok=False,err='请先结束行走地图更新，再编辑地图')),'')
        with patch('bridge.map_service.subprocess.run',return_value=result):
            with self.assertRaises(ValueError) as error:service.privileged('catalog-delete',dict(name='new'))
        self.assertEqual(str(error.exception),'请先结束行走地图更新，再编辑地图')

    def test_purge_requires_exact_confirmation_and_preserves_external_link_targets(self):
        external=self.root/'external';external.write_text('keep')
        (self.maps/'new/external-link').symlink_to(external)
        deleted=self.catalog.execute('delete',dict(name='new'));key=deleted['trash_id']
        with self.assertRaises(ValueError):self.catalog.execute('purge',dict(id=key,confirmed_name='wrong'))
        self.assertTrue((self.catalog.trash/key/'map/full_cloud.pcd').is_file())
        result=self.catalog.execute('purge',dict(id=key,confirmed_name='new'))
        self.assertTrue(result['permanently_deleted']);self.assertFalse((self.catalog.trash/key).exists());self.assertEqual(external.read_text(),'keep')

    def test_purge_still_blocks_busy_state_and_active_references(self):
        deleted=self.catalog.execute('delete',dict(name='new'));key=deleted['trash_id']
        busy=MapCatalog(self.maps,lambda:(_ for _ in ()).throw(ValueError('busy')))
        with self.assertRaisesRegex(ValueError,'busy'):busy.execute('purge',dict(id=key,confirmed_name='new'))
        (self.maps/'old/linked-map').symlink_to(self.catalog.trash/key/'map/full_cloud.pcd')
        with self.assertRaisesRegex(ValueError,'引用'):self.catalog.execute('purge',dict(id=key,confirmed_name='new'))


if __name__=='__main__':unittest.main()
