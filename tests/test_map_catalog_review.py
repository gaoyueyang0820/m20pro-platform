"""Map-management failure/regression cases use only temporary synthetic data."""
import json
from pathlib import Path
import tempfile
import concurrent.futures
import threading
import unittest
from unittest.mock import patch

from map_catalog import MapCatalog


class MapCatalogReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.maps=Path(self.temp.name)/'maps';self.maps.mkdir()
        for name in ('active-map','old','candidate'):
            folder=self.maps/name;folder.mkdir()
            (folder/'full_cloud.pcd').write_bytes(b'synthetic-cloud')
            (folder/'occ_grid.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            (folder/'occ_grid.yaml').write_text('image: occ_grid.pgm\n')
        (self.maps/'active').symlink_to('active-map',target_is_directory=True)
        self.catalog=MapCatalog(self.maps)
    def activate(self,name):
        (self.maps/'active').unlink();(self.maps/'active').symlink_to(name,target_is_directory=True)
    def merge_record(self):
        (self.maps/'candidate/.m20-project.json').write_text(json.dumps(dict(sessions=[dict(map='candidate',base_map='old',includes_complete_base=True,created=1)])))
        self.activate('candidate')

    def test_purge_failure_keeps_identity_and_is_not_restorable(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        record=self.catalog.trash/key;original=record/'map/full_cloud.pcd'
        def fail_after_one_file(target,*args,**kwargs):
            # The old implementation invokes rmtree on the whole record, so its
            # meta.json can disappear before a later payload I/O failure.
            target=Path(target)
            if (target/'meta.json').exists():(target/'meta.json').unlink()
            original.unlink()
            raise OSError('synthetic disk I/O failure')
        with patch('map_catalog.shutil.rmtree',side_effect=fail_after_one_file):
            with self.assertRaises((OSError,ValueError)):
                self.catalog.execute('purge',dict(id=key,confirmed_name='old'))
        self.assertTrue((record/'meta.json').is_file(),'failed purge lost the only identity record')
        rows=self.catalog.trash_list();self.assertEqual(rows[0]['name'],'old')
        self.assertFalse(rows[0]['recoverable'])
        with self.assertRaisesRegex(ValueError,'永久删除|部分|恢复'):
            self.catalog.execute('restore',dict(id=key))
        self.assertTrue(self.catalog.execute('purge',dict(id=key,confirmed_name='old'))['permanently_deleted'])

    def test_reference_from_another_recycled_map_blocks_delete(self):
        (self.maps/'candidate/source').symlink_to(self.maps/'old/full_cloud.pcd')
        self.catalog.execute('delete',dict(name='candidate'))
        with self.assertRaisesRegex(ValueError,'引用'):
            self.catalog.execute('delete',dict(name='old'))
        self.assertTrue((self.maps/'old/full_cloud.pcd').exists())

    def test_yaml_and_toml_file_references_block_delete(self):
        for filename,body in [('occ_grid.yaml','image: '+str(self.maps/'old/occ_grid.pgm')+'\n'),
                              ('occ_grid_id_map.toml','[maps]\npath = '+json.dumps(str(self.maps/'old')+'/')+'\n')]:
            with self.subTest(filename=filename):
                target=self.maps/'candidate'/filename;old=target.read_bytes() if target.exists() else None
                target.write_text(body)
                with self.assertRaisesRegex(ValueError,'引用'):
                    self.catalog.execute('delete',dict(name='old'))
                if old is None:target.unlink()
                else:target.write_bytes(old)

    def test_candidate_with_truncated_raw_scan_cannot_cleanup_old(self):
        for name,data in [('old',b'original synthetic lidar samples'),('candidate',b'')]:
            folder=self.maps/name/'.sessions/session_0';folder.mkdir(parents=True)
            (folder/'scan.pcd').write_bytes(data)
        self.merge_record()
        with self.assertRaisesRegex(ValueError,'完整|变化|不一致'):
            self.catalog.execute('cleanup-previous',dict(name='candidate',base='old'))
        self.assertTrue((self.maps/'old/.sessions/session_0/scan.pcd').exists())

    def test_reference_from_recycled_relative_link_blocks_delete(self):
        (self.maps/'candidate/source').symlink_to('../old/full_cloud.pcd')
        self.catalog.execute('delete',dict(name='candidate'))
        with self.assertRaisesRegex(ValueError,'引用'):
            self.catalog.execute('delete',dict(name='old'))

    def test_stable_source_history_is_not_a_resource_reference(self):
        self.merge_record()
        (self.maps/'candidate/history.yaml').write_text('base_map: old\nsource_map: old\nimage: '+str(self.maps/'old/occ_grid.pgm')+'\n')
        result=self.catalog.execute('cleanup-previous',dict(name='candidate',base='old'))
        self.assertTrue(result['recoverable'])

    def test_same_size_corruption_in_raw_scan_cannot_cleanup(self):
        for name,data in [('old',b'original'),('candidate',b'corrupt!')]:
            folder=self.maps/name/'.sessions/session_0';folder.mkdir(parents=True)
            (folder/'scan.pcd').write_bytes(data)
        self.merge_record()
        with self.assertRaisesRegex(ValueError,'完整|不一致'):
            self.catalog.execute('cleanup-previous',dict(name='candidate',base='old'))

    def test_valid_raw_copy_and_regenerated_grid_can_cleanup(self):
        for name in ('old','candidate'):
            folder=self.maps/name/'.sessions/session_0';folder.mkdir(parents=True)
            (folder/'scan.pcd').write_bytes(b'original synthetic lidar points')
        (self.maps/'candidate/occ_grid.pgm').write_bytes(b'new fused grid')
        self.merge_record()
        result=self.catalog.execute('cleanup-previous',dict(name='candidate',base='old'))
        self.assertTrue(result['recoverable'])

    def test_purge_retry_never_deletes_recreated_same_name(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        self.assertTrue(self.catalog.execute('purge',dict(id=key,confirmed_name='old'))['permanently_deleted'])
        new=self.maps/'old';new.mkdir();(new/'new-data').write_text('new incarnation')
        again=self.catalog.execute('purge',dict(id=key,confirmed_name='old'))
        self.assertTrue(again['already_completed'])
        self.assertEqual((new/'new-data').read_text(),'new incarnation')
        with self.assertRaisesRegex(ValueError,'确认'):
            self.catalog.execute('purge',dict(id=key,confirmed_name='candidate'))

    def test_restore_retry_is_bound_to_original_directory_identity(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        self.catalog.execute('restore',dict(id=key))
        self.assertTrue(self.catalog.execute('restore',dict(id=key))['already_completed'])
        self.catalog.execute('delete',dict(name='old'))
        (self.maps/'old').mkdir();(self.maps/'old/replacement').write_text('keep')
        with self.assertRaisesRegex(ValueError,'变化|未覆盖'):
            self.catalog.execute('restore',dict(id=key))
        self.assertEqual((self.maps/'old/replacement').read_text(),'keep')

    def test_restore_metadata_write_failure_keeps_restored_map_listable(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        with patch('map_catalog.atomic_json',side_effect=OSError('synthetic metadata failure')):
            restored=self.catalog.execute('restore',dict(id=key))
        self.assertTrue(restored['ok'])
        self.assertTrue((self.maps/'old/full_cloud.pcd').exists())
        self.assertEqual(self.catalog.trash_list(),[])
        self.assertTrue(self.catalog.execute('restore',dict(id=key))['already_completed'])

    def test_archive_rename_failure_preserves_source_and_leaves_no_false_recycle(self):
        with patch('pathlib.Path.rename',side_effect=OSError('synthetic rename failure')):
            with self.assertRaises(OSError):self.catalog.execute('delete',dict(name='old'))
        self.assertEqual((self.maps/'old/full_cloud.pcd').read_bytes(),b'synthetic-cloud')
        self.assertEqual(self.catalog.trash_list(),[])

    def test_replaced_recycle_payload_is_not_owned_by_old_request(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        folder=self.catalog.trash/key/'map';folder.rename(self.catalog.trash/key/'retained-original')
        folder.mkdir();(folder/'unrelated').write_text('keep')
        with self.assertRaisesRegex(ValueError,'归属'):
            self.catalog.execute('purge',dict(id=key,confirmed_name='old'))
        self.assertEqual((folder/'unrelated').read_text(),'keep')

    def test_catalog_operations_serialize_concurrent_delete_and_restore(self):
        gate=threading.Barrier(2)
        def attempt():
            gate.wait()
            try:return self.catalog.execute('delete',dict(name='old'))
            except ValueError:return None
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            results=list(pool.map(lambda _:attempt(),range(2)))
        successful=[r for r in results if r]
        self.assertEqual(len(successful),1)
        self.assertEqual(len(self.catalog.trash_list()),1)
        self.catalog.execute('restore',dict(id=successful[0]['trash_id']))
        self.assertEqual((self.maps/'old/full_cloud.pcd').read_bytes(),b'synthetic-cloud')

    def test_trash_or_payload_symlink_cannot_target_external_files(self):
        result=self.catalog.execute('delete',dict(name='old'));key=result['trash_id']
        source=self.catalog.trash/key;payload=source/'map'
        external=Path(self.temp.name)/'external';external.mkdir();(external/'keep').write_text('owned elsewhere')
        payload.rename(source/'original');payload.symlink_to(external,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'路径'):
            self.catalog.execute('purge',dict(id=key,confirmed_name='old'))
        self.assertEqual((external/'keep').read_text(),'owned elsewhere')

    def test_archive_metadata_failure_does_not_hide_or_move_source(self):
        with patch('map_catalog.atomic_json',side_effect=OSError('synthetic journal I/O failure')):
            with self.assertRaises(OSError):self.catalog.execute('delete',dict(name='old'))
        self.assertEqual((self.maps/'old/full_cloud.pcd').read_bytes(),b'synthetic-cloud')
        self.assertEqual(self.catalog.trash_list(),[])

    def test_interrupted_empty_journal_does_not_break_library_or_future_edits(self):
        self.catalog.trash.mkdir();(self.catalog.trash/('a'*32)).mkdir()
        self.assertEqual(self.catalog.trash_list(),[])
        self.assertTrue(self.catalog.execute('delete',dict(name='old'))['recoverable'])


if __name__=='__main__':unittest.main()
