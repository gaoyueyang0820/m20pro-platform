# coding: utf-8
import hashlib,json,sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.map_fusion import read_pcd,write_pcd,pose_matrix,transform,fuse_grid,read_grid,fuse_maps,validate_alignment
from bridge.mapping_job import mapping_phase

def fixture(folder,points,origin=(0,0),grid=None):
    folder.mkdir();write_pcd(folder/'full_cloud.pcd',points)
    session=folder/'.sessions/session_0';(session/'lidar_cloud').mkdir(parents=True)
    write_pcd(session/'lidar_cloud/0.pcd',points[:300]);(session/'poses.txt').write_text('1 0 0 0 0 0 0 1\n')
    (folder/'.optimizers/optimizer_0').mkdir(parents=True);(folder/'.optimizers/optimizer_0/loops.txt').write_text('0 0 0 0 0 0 0 0 1\n')
    image=grid if grid is not None else np.array([[0,254],[254,254]],dtype=np.uint8)
    (folder/'occ_grid.pgm').write_bytes(b'P5\n2 2\n255\n'+image.tobytes())
    (folder/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 1\norigin: [%s,%s,0]\n'%origin)
    (folder/'occ_grid_id_map.toml').write_text('path = "old"\n[[grids]]\nid=0\nname="occ_grid"\n')

class FusionTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def test_pcd_roundtrip_preserves_all_points_and_intensity(self):
        points=np.random.default_rng(4).normal(size=(5471,4)).astype(np.float32);p=self.root/'all.pcd';write_pcd(p,points)
        np.testing.assert_array_equal(read_pcd(p),points)
    def test_grid_expands_without_rescanning_or_erasing_old_cells(self):
        old=dict(image=np.array([[0,254],[254,205]],dtype=np.uint8),resolution=1,origin=[0,0,0])
        new=dict(image=np.full((2,2),254,dtype=np.uint8),resolution=1,origin=[1,0,0])
        merged=fuse_grid(old,new,np.eye(4))
        self.assertEqual(merged['image'].shape,(2,3));self.assertEqual(merged['image'][0,0],0);self.assertEqual(merged['image'][1,2],254)
    def test_repeated_slam_packages_cannot_clear_wall_without_independent_evidence(self):
        old=dict(image=np.array([[0]],dtype=np.uint8),resolution=1,origin=[0,0,0]);new=dict(image=np.array([[254]],dtype=np.uint8),resolution=1,origin=[0,0,0])
        prior=None
        for i in range(4):
            result=fuse_grid(old,new,np.eye(4),prior);prior=result['score'];old['image']=result['image']
            if i==0:self.assertEqual(result['image'][0,0],0)
        self.assertEqual(result['image'][0,0],0)
        result=fuse_grid(old,new,np.eye(4),prior,confirmed_changes={(0,0):254})
        self.assertEqual(result['image'][0,0],254)
    def test_one_conflicting_scan_does_not_turn_known_free_space_unknown(self):
        old=dict(image=np.array([[254]],dtype=np.uint8),resolution=1,origin=[0,0,0]);new=dict(image=np.array([[0]],dtype=np.uint8),resolution=1,origin=[0,0,0])
        result=fuse_grid(old,new,np.eye(4))
        self.assertEqual(result['image'][0,0],254);self.assertEqual(result['deferred_known_conflicts'],1)
    def test_floor_overlap_cannot_publish_a_displaced_wall(self):
        x,y=np.meshgrid(np.linspace(-3,3,40),np.linspace(-3,3,40));floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55),np.ones(x.size)])
        yy,zz=np.meshgrid(np.linspace(-2,2,40),np.linspace(-.2,1.4,20));wall=np.column_stack([np.full(yy.size,2.),yy.ravel(),zz.ravel(),np.ones(yy.size)])
        base=self.root/'base';fresh=self.root/'fresh';out=self.root/'bad'
        fixture(base,np.vstack([wall,floor]));fixture(fresh,np.vstack([wall+[.6,0,0,0],floor]))
        write_pcd(fresh/'.sessions/session_0/lidar_cloud/0.pcd',np.vstack([wall+[.6,0,0,0],floor]))
        with self.assertRaisesRegex(ValueError,'墙体结构'):fuse_maps(base,fresh,out,dict(x=0,y=0,z=0,yaw=0),registration=lambda a,b:np.eye(4))
        self.assertFalse(out.exists())
    def test_manual_grid_edits_override_older_cached_evidence(self):
        old=dict(image=np.array([[0]],dtype=np.uint8),resolution=1,origin=[0,0,0]);fresh=dict(image=np.array([[205]],dtype=np.uint8),resolution=1,origin=[0,0,0])
        result=fuse_grid(old,fresh,np.eye(4),np.array([[-5.]],dtype=np.float32))
        self.assertEqual(result['image'][0,0],0)
    def test_imported_grid_negate_is_normalized_before_fusion(self):
        points=np.ones((4,4),dtype=np.float32);folder=self.root/'inverted';fixture(folder,points,grid=np.array([[0,254],[254,0]],dtype=np.uint8))
        p=folder/'occ_grid.yaml';p.write_text(p.read_text()+'negate: 1\n')
        image=read_grid(folder)['image'];self.assertEqual(image[0,0],254);self.assertEqual(image[0,1],0)
    def test_bad_alignment_rejected(self):
        rng=np.random.default_rng(7);cloud=rng.normal(size=(300,4));far=cloud.copy();far[:,:3]+=10
        with self.assertRaises(ValueError):validate_alignment(cloud,far,np.eye(4))
        bad=np.eye(4);bad[0,3]=5
        with self.assertRaises(ValueError):validate_alignment(cloud,cloud,bad)
    def test_fusion_keeps_original_and_raw_history_and_adds_a_version(self):
        rng=np.random.default_rng(3);points=rng.normal(size=(700,4)).astype(np.float32)
        base=self.root/'base';fresh=self.root/'new';out=self.root/'base-r2';fixture(base,points);fixture(fresh,np.concatenate([points,points[:100]+np.array([5,0,0,0])]),(1,0))
        digest=hashlib.sha256((base/'full_cloud.pcd').read_bytes()).hexdigest()
        result=fuse_maps(base,fresh,out,dict(x=0,y=0,z=0,yaw=0),registration=lambda a,b:np.eye(4))
        self.assertEqual(result['revision'],2);self.assertGreater(read_pcd(out/'full_cloud.pcd')[:,0].max(),4)
        self.assertEqual(hashlib.sha256((base/'full_cloud.pcd').read_bytes()).hexdigest(),digest)
        self.assertEqual((out/'.sessions/session_0/lidar_cloud/0.pcd').read_bytes(),(base/'.sessions/session_0/lidar_cloud/0.pcd').read_bytes())
        self.assertTrue((out/'.sessions/session_1/map_transform.json').is_file());self.assertFalse((out/'.blocks').exists())
        self.assertEqual(read_grid(out)['image'].shape,(2,3));self.assertEqual(json.loads((out/'.m20-project.json').read_text())['root_map'],'base')
        self.assertTrue(result['report']['includes_complete_base'])
    def test_fusion_keeps_each_unobserved_old_point_without_voxel_averaging(self):
        points=np.random.default_rng(13).normal(size=(400,4)).astype(np.float32)
        remote=np.array([[30,30,0,12],[30.001,30.001,0,25]],dtype=np.float32)
        base=self.root/'base';fresh=self.root/'new';out=self.root/'result'
        fixture(base,np.concatenate([points,remote]));fixture(fresh,points)
        result=fuse_maps(base,fresh,out,dict(x=0,y=0,z=0,yaw=0),registration=lambda a,b:np.eye(4))
        np.testing.assert_array_equal(read_pcd(out/'full_cloud.pcd')[read_pcd(out/'full_cloud.pcd')[:,0]>20],remote)
        self.assertGreaterEqual(result['report']['preserved_unobserved_points'],2)
    def test_map_project_reports_robot_component_readiness(self):
        from bridge.map_service import MapService
        from unittest.mock import patch
        base=self.root/'baseline';fixture(base,np.ones((4,4),dtype=np.float32));service=MapService(maps=self.root,data=self.root/'agent')
        with patch.object(service,'privileged',return_value=dict(ready=False,pipeline_version=2,message='dependency missing')):
            result=service.dispatch('/maps/project',dict(name='baseline'))
        self.assertFalse(result['extension_ready']);self.assertEqual(result['project']['revision'],1)

    def test_failed_alignment_never_publishes_candidate(self):
        points=np.random.default_rng(9).normal(size=(400,4)).astype(np.float32);base=self.root/'base';fresh=self.root/'new';out=self.root/'result';fixture(base,points);fixture(fresh,points)
        def fail(a,b):raise ValueError('no overlap')
        with self.assertRaises(ValueError):fuse_maps(base,fresh,out,dict(x=0,y=0,z=0,yaw=0),registration=fail)
        self.assertFalse(out.exists());self.assertTrue((base/'full_cloud.pcd').is_file())
    def test_extension_waits_for_fusion_instead_of_reporting_raw_scan_saved(self):
        start=dict(started=100,name='m20-session-a',context=dict(workflow='extension'))
        stop=dict(state='command_finished',started=110)
        maps=[dict(name='m20-session-a-123',modified=120,has_cloud=True,grids=[0])]
        self.assertEqual(mapping_phase(start,stop,maps)['phase'],'saving')
        stop.update(result_maps=['base-r2'],fusion=dict(keyframes=10))
        result=mapping_phase(start,stop,maps);self.assertEqual(result['phase'],'saved');self.assertEqual(result['result_maps'],['base-r2'])

class ExtensionWorkerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.logs=self.root/'logs';self.logs.mkdir();self.maps=self.root/'maps';self.maps.mkdir()
        (self.maps/'base').mkdir();scan=self.maps/'capture-123';scan.mkdir();(scan/'full_cloud.pcd').write_bytes(b'fixture')
        self.start=dict(state='command_finished',started=100,name='capture',context=dict(workflow='extension',base='base',maps=str(self.maps),anchor=dict(x=0,y=0,z=0,yaw=0),grid_id=0))
        (self.logs/'mapping.json').write_text(json.dumps(self.start))
    def tearDown(self):self.tmp.cleanup()
    def test_stop_fuses_candidate_without_automatic_activation(self):
        from bridge import mapping_job
        from unittest.mock import patch
        from types import SimpleNamespace
        calls=[]
        def run(argv,**kwargs):calls.append(argv);return SimpleNamespace(returncode=0,stdout='',stderr='')
        def fuse(base,source,out,*args):out.mkdir();return dict(ok=True,report=dict(base_map='base'),revision=2)
        with patch('bridge.mapping_job.subprocess.run',side_effect=run),patch('m20.map_fusion.fuse_maps',side_effect=fuse):mapping_job.worker('stop_mapping',self.logs)
        result=mapping_job.read_job('stop_mapping',self.logs)
        self.assertEqual(result['state'],'command_finished');self.assertEqual(result['revision'],2);self.assertEqual(calls,[['drmap','stop_mapping']]);self.assertFalse(result['activated'])
        self.assertEqual(len(result['result_maps']),1)
    def test_failed_fusion_does_not_activate_an_incomplete_map(self):
        from bridge import mapping_job
        from unittest.mock import patch
        from types import SimpleNamespace
        with patch('bridge.mapping_job.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run,patch('m20.map_fusion.fuse_maps',side_effect=ValueError('bad overlap')):
            mapping_job.worker('stop_mapping',self.logs)
        result=mapping_job.read_job('stop_mapping',self.logs);self.assertEqual(result['state'],'failed');self.assertIn('bad overlap',result['message']);self.assertEqual(run.call_count,1)
    def test_repeat_stop_returns_saved_result_without_starting_another_worker(self):
        from bridge import mapping_job
        from unittest.mock import patch
        (self.logs/'stop_mapping.json').write_text(json.dumps(dict(state='command_finished',started=110,result_maps=['base-r2'])))
        with patch('bridge.mapping_job.subprocess.Popen') as process:result=mapping_job.launch('stop_mapping',logs=self.logs)
        process.assert_not_called();self.assertEqual(result['delivery'],'already_saved');self.assertEqual(result['result_maps'],['base-r2'])

class NativeSessionChangeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        axis=np.arange(-2.4,2.41,.12);x,y=np.meshgrid(axis,axis)
        floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size),np.ones(x.size)])
        horizontal=np.arange(-2.4,2.41,.12);height=np.arange(.2,1.5,.12);yy,zz=np.meshgrid(horizontal,height)
        wallx=np.column_stack([np.full(yy.size,2.5),yy.ravel(),zz.ravel(),np.ones(yy.size)])
        wally=wallx[:,[1,0,2,3]].copy();wally[:,1]=-2.5
        self.fixed=np.vstack([floor,wallx,wally])
        y,z=np.meshgrid([-.06,0,.06],np.arange(.2,1.1,.1));self.object=np.column_stack([np.full(y.size,1.5),y.ravel(),z.ravel(),np.ones(y.size)])
    def tearDown(self):self.tmp.cleanup()
    def native(self,name,present,stamps=None):
        folder=self.root/name;(folder/'.sessions/session_0/lidar_cloud').mkdir(parents=True)
        poses=[];world=[];image=np.full((40,40),254,np.uint8)
        for index,exists in enumerate(present):
            points=np.vstack([self.fixed,self.object]) if exists else self.fixed
            pose=[-index*.4,0,.55,0,0,0,1];stamp=100+index*10 if stamps is None else stamps[index]
            poses.append([stamp]+pose);world.append(points)
            body=transform(points,np.linalg.inv(pose_matrix(pose)));write_pcd(folder/('.sessions/session_0/lidar_cloud/%d.pcd'%index),body)
        np.savetxt(folder/'.sessions/session_0/poses.txt',np.array(poses),fmt='%.9f')
        write_pcd(folder/'full_cloud.pcd',np.vstack(world))
        if present[-1]:
            for p in self.object:image[39-int(np.floor((p[1]+4)/.2)),int(np.floor((p[0]+4)/.2))]=0
        (folder/'occ_grid.pgm').write_bytes(b'P5\n40 40\n255\n'+image.tobytes())
        (folder/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.2\norigin: [-4,-4,0]\n')
        return folder
    def fuse(self,base,session):
        out=self.root/'candidate';r=fuse_maps(base,session,out,dict(x=0,y=0,z=.55,yaw=0),registration=lambda a,b:np.eye(4))
        return out,r
    def object_points(self,folder):
        p=read_pcd(folder/'full_cloud.pcd');return p[(abs(p[:,0]-1.5)<.08)&(abs(p[:,1])<.1)&(p[:,2]>.15)]
    def test_same_extension_confirms_new_obstacle_from_saved_keyframes(self):
        base=self.native('base',[False]);session=self.native('scan',[True]*3)
        out,r=self.fuse(base,session)
        self.assertGreater(r['report']['confirmed_known_changes'],0)
        self.assertGreater(len(self.object_points(out)),0)
        self.assertEqual(r['report']['temporal_evidence']['source'],'saved_slam_keyframes')
        self.assertEqual(r['report']['temporal_evidence']['frames'],3)
    def test_same_extension_clears_old_obstacle_without_adding_early_ghost_back(self):
        base=self.native('base',[True]);session=self.native('scan',[True]+[False]*5)
        out,r=self.fuse(base,session)
        self.assertGreater(r['report']['removed_confirmed_points'],0)
        self.assertEqual(len(self.object_points(out)),0)
        self.assertEqual(read_grid(out)['image'][19,27],254)
        self.assertGreater(len(self.object_points(base)),0)
    def test_one_clear_frame_never_erases_old_wall(self):
        base=self.native('base',[True]);session=self.native('scan',[False])
        out,r=self.fuse(base,session)
        self.assertEqual(r['report']['removed_confirmed_points'],0)
        self.assertGreater(len(self.object_points(out)),0)
        self.assertEqual(read_grid(out)['image'][19,27],0)
    def test_frame_count_without_time_diversity_cannot_promote_new_object(self):
        base=self.native('base',[False]);session=self.native('scan',[True]*3,[100,100.1,100.2])
        out,r=self.fuse(base,session)
        self.assertEqual(r['report']['confirmed_known_changes'],0)
        self.assertEqual(len(self.object_points(out)),0)
    def test_frame_pose_matrix_roundtrip_keeps_rotation_and_translation(self):
        from m20.map_refinement import matrix_pose
        T=pose_matrix([3,2,.5,.1,.2,.3,.92]);np.testing.assert_allclose(pose_matrix(matrix_pose(T)),T,atol=1e-9)
    def test_long_dense_batch_defers_without_silent_keyframe_sampling(self):
        from m20.map_refinement import session_evidence
        from unittest.mock import patch
        poses=np.tile([0,0,0,.55,0,0,0,1.],(121,1));poses[:,0]=np.arange(121)
        with patch('m20.map_refinement.read_pcd') as read:
            evidence,report=session_evidence(self.root/'not-read',poses,np.eye(4),{})
        self.assertIsNone(evidence);self.assertEqual(report['reason'],'keyframe_budget_exceeded');read.assert_not_called()
    def test_nonmonotonic_native_timestamps_cannot_become_repeated_visits(self):
        from m20.map_refinement import session_evidence
        poses=np.array([[100,0,0,.55,0,0,0,1],[99,0,0,.55,0,0,0,1],[101,0,0,.55,0,0,0,1]],float)
        with self.assertRaisesRegex(ValueError,'时间非单调'):session_evidence(self.root/'not-read',poses,np.eye(4),{})

if __name__=='__main__':unittest.main()
