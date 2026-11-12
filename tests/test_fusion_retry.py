import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from bridge.mapping_job import retry_context, worker
from bridge.root_actions import _map_readable
from m20.map_fusion import voxel


class FusionRetryTests(unittest.TestCase):
    def test_root_import_makes_regular_files_readable_without_following_links(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'candidate';folder.mkdir(mode=0o700)
            cloud=folder/'full_cloud.pcd';cloud.touch(mode=0o600)
            private=Path(temp)/'outside';private.touch(mode=0o600)
            (folder/'link').symlink_to(private)
            _map_readable(folder)
            self.assertEqual(folder.stat().st_mode&0o777,0o755)
            self.assertEqual(cloud.stat().st_mode&0o777,0o644)
            self.assertEqual(private.stat().st_mode&0o777,0o600)

    def test_nos_numpy_does_not_receive_an_empty_axis_unique(self):
        points=np.empty((0,4),np.float32)
        with patch('numpy.unique',side_effect=ValueError('cannot reshape array of size 0 into shape (0,newaxis)')):
            self.assertEqual(voxel(points).shape,(0,4))

    def test_retry_uses_retained_input_and_never_calls_vendor_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            logs=Path(temp)/'logs';logs.mkdir()
            maps=Path(temp)/'maps';maps.mkdir()
            base=maps/'original';base.mkdir()
            (maps/'active').symlink_to(base.name)
            session=maps/'m20-session-fixture-20260101';session.mkdir()
            (session/'full_cloud.pcd').write_bytes(b'original scan')
            (session/'.sessions/session_0').mkdir(parents=True)
            (session/'.sessions/session_0/poses.txt').write_text('0 0 0 0 0 0 0 1')
            context=dict(workflow='extension',base=base.name,maps=str(maps),anchor=dict(x=0,y=0,z=0,yaw=0),grid_id=0)
            capture=dict(state='command_finished',name='m20-session-fixture',started=1,context=context,request_id='a'*32)
            stop=dict(state='failed',started=2,context=context,vendor_save_succeeded=True,observation_session=dict(name=session.name))
            (logs/'mapping.json').write_text(json.dumps(capture))
            self.assertRaises(ValueError,retry_context,dict(capture,state='running'),stop,maps)
            prepared=retry_context(capture,stop,maps)
            def fuse(old,saved,output,*args):
                self.assertEqual((old,saved),(base,session))
                output.mkdir()
                return dict(report={},revision=2)
            with patch('bridge.mapping_job.subprocess.run',side_effect=AssertionError('retry must not run drmap')),patch('m20.map_fusion.fuse_maps',side_effect=fuse):
                worker('stop_mapping',logs,context=prepared,expected_start_request_id=capture['request_id'])
            result=json.loads((logs/'stop_mapping.json').read_text())
            self.assertEqual(result['fusion_status'],'candidate_created')
            self.assertTrue((maps/result['result_maps'][0]).is_dir())
            self.assertEqual((maps/'active').resolve(),base.resolve())
            self.assertEqual((session/'full_cloud.pcd').read_bytes(),b'original scan')
