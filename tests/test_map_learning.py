import base64,json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_refinement import collect_evidence,refine_map,base_fingerprint
from map_fusion import write_pcd,read_grid
from bridge.map_service import MapService
from bridge.map_refinement_agent import RefinementAgent

class MapLearningTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.maps=self.root/'maps';self.base=self.maps/'original';self.base.mkdir(parents=True);self.session=self.root/'session';self.session.mkdir()
        self.image=np.full((40,40),254,np.uint8);self.write_grid(self.image)
        axis=np.arange(-3,3.01,.1);x,y=np.meshgrid(axis,axis);self.floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]).astype(np.float32)
        yy,zz=np.meshgrid(np.arange(-.3,.31,.05),np.arange(.2,1.3,.1));self.wall=np.column_stack([np.full(yy.size,1.5),yy.ravel(),zz.ravel()]).astype(np.float32)
        write_pcd(self.base/'full_cloud.pcd',self.floor);(self.maps/'active').symlink_to('original')
    def tearDown(self):self.tmp.cleanup()
    def write_grid(self,image):
        (self.base/'occ_grid.pgm').write_bytes(b'P5\n40 40\n255\n'+image.tobytes())
        (self.base/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.2\norigin: [-4,-4,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    def frame(self,index,wall=True,stamp=None,pose=None):
        p=self.session/('frame-%06d.npz'%index);pts=np.vstack([self.floor,self.wall]) if wall else self.floor
        pose=pose or [index*.3,0,.55,0,0,0,1]
        np.savez_compressed(p,points=pts,pose=pose,stamp=index*10 if stamp is None else stamp);return p
    def test_single_visit_or_stationary_frames_do_not_confirm_changes(self):
        files=[self.frame(i,stamp=i*.1) for i in range(5)]
        e=collect_evidence(files,read_grid(self.base));self.assertEqual(len(e['points']),0);self.assertEqual(e['occupied_added'],0)
        files=[self.frame(i,pose=[0,0,.55,0,0,0,1]) for i in range(5)]
        e=collect_evidence(files,read_grid(self.base));self.assertEqual(len(e['points']),0)
    def test_transient_obstacle_not_added_but_repeated_geometry_is(self):
        files=[self.frame(i,wall=i==0) for i in range(3)];e=collect_evidence(files,read_grid(self.base))
        self.assertEqual(e['occupied_added'],0);self.assertTrue(np.all(e['points'][:,2]<.1))
        files=[self.frame(i) for i in range(3)];e=collect_evidence(files,read_grid(self.base))
        self.assertGreater(e['occupied_added'],0);self.assertTrue((e['points'][:,2]>.2).any())
    def test_observed_free_space_needs_five_visits_before_clearing_old_obstacle(self):
        self.image[19,12]=0;self.write_grid(self.image)
        e=collect_evidence([self.frame(i,wall=False) for i in range(3)],read_grid(self.base));self.assertEqual(e['image'][19,12],0)
        e=collect_evidence([self.frame(i,wall=False) for i in range(5)],read_grid(self.base));self.assertEqual(e['image'][19,12],254)
    def test_long_ago_obstacle_does_not_permanently_block_confirmed_removal(self):
        gx=int((1.5+4)/.2);gy=39-int((0+4)/.2);self.image[gy,gx]=0;self.write_grid(self.image)
        files=[self.frame(0,wall=True)]+[self.frame(i,wall=False,pose=[-i*.3,0,.55,0,0,0,1]) for i in range(1,6)]
        e=collect_evidence(files,read_grid(self.base));self.assertEqual(e['image'][gy,gx],254)
    def test_more_time_from_one_view_does_not_make_new_obstacle_permanent(self):
        files=[self.frame(i,pose=[(i%2)*.1,0,.55,0,0,0,1]) for i in range(6)]
        e=collect_evidence(files,read_grid(self.base));self.assertEqual(e['occupied_added'],0)
        self.assertFalse((e['points'][:,2]>.2).any())
    def test_object_seen_for_three_epochs_then_removed_does_not_leave_ghost_cloud(self):
        files=[self.frame(i,wall=i<3,pose=[-i*.3,0,.55,0,0,0,1]) for i in range(8)]
        e=collect_evidence(files,read_grid(self.base))
        self.assertEqual(e['occupied_added'],0);self.assertFalse((e['points'][:,2]>.2).any())
    def test_consistently_shifted_wall_is_not_treated_as_environment_change(self):
        write_pcd(self.base/'full_cloud.pcd',np.vstack([self.floor,self.wall]))
        for i in range(5):
            pts=np.vstack([self.floor,self.wall+[.6,0,0]])
            np.savez_compressed(self.session/('frame-%06d.npz'%i),points=pts,pose=[i*.3,0,.55,0,0,0,1],stamp=i*10)
        out=self.maps/'bad-update'
        with self.assertRaisesRegex(ValueError,'旧墙体结构'):refine_map(self.base,self.session,out,0,base_fingerprint(self.base))
        self.assertFalse(out.exists())
    def test_generated_revision_never_modifies_or_activates_original(self):
        fingerprint=base_fingerprint(self.base)
        for i in range(3):self.frame(i)
        result=refine_map(self.base,self.session,self.maps/'updated',0,fingerprint)
        self.assertFalse(result['activated']);self.assertEqual(base_fingerprint(self.base),fingerprint)
        self.assertEqual((self.maps/'active').resolve(),self.base.resolve())
        self.assertTrue((self.maps/'updated/full_cloud.pcd').is_file());self.assertGreater(result['report']['stable_points'],0)
        with self.assertRaisesRegex(ValueError,'原地图内容已变化'):refine_map(self.base,self.session,self.maps/'other',0,'stale')
    def test_recorder_quality_and_duplicate_stamp_checks(self):
        service=MapService(self.maps,self.root/'data',demo=True);a=RefinementAgent(service,lambda:None)
        with patch('threading.Thread.start'):a.control({'action':'enable'})
        def packet(stamp,x):return dict(protocol=1,frame_id='map',age=0,quality=[.05,.99],pose=[x,0,.55,0,0,0,1],stamp=stamp,points=base64.b64encode(self.floor.astype('<f4').tobytes()).decode())
        self.assertTrue(a.ingest(packet(100,0)));self.assertFalse(a.ingest(packet(100,0)));self.assertFalse(a.ingest(packet(110,0)))
        self.assertTrue(a.ingest(packet(120,.5)));bad=packet(130,1);bad['quality']=[.5,.3]
        with self.assertRaisesRegex(ValueError,'质量'):a.ingest(bad)
        self.assertEqual(a.row['frames'],2)
        restored=RefinementAgent(service,lambda:None);self.assertEqual(restored.row['frames'],2);self.assertFalse(restored.row['enabled'])
        persisted=json.loads(restored.file.read_text());self.assertFalse(persisted['enabled']);self.assertEqual(persisted['state'],'off');self.assertEqual(persisted['frames'],2)
        a.close();restored.close()
    def test_complete_old_map_is_retained_while_confirmed_new_area_expands_grid(self):
        from map_fusion import read_pcd
        far=np.array([[-3.8,-3.8,1.2,77.]],dtype=np.float32)
        old=np.column_stack([self.floor,np.full(len(self.floor),12.)]).astype(np.float32)
        write_pcd(self.base/'full_cloud.pcd',np.vstack([old,far]))
        self.image[0,0]=0;self.write_grid(self.image)
        historical=self.base/'.sessions/session_0';historical.mkdir(parents=True);(historical/'poses.txt').write_text('retained original trajectory')
        for i in range(5):
            np.savez_compressed(self.session/('frame-%06d.npz'%i),points=self.floor+[4,0,0],pose=[3+i*.3,0,.55,0,0,0,1],stamp=i*10)
        before=base_fingerprint(self.base);destination=self.maps/'expanded'
        r=refine_map(self.base,self.session,destination,0,before);new=read_pcd(destination/'full_cloud.pcd');grid=read_grid(destination)
        self.assertTrue(r['report']['includes_complete_base']);self.assertGreater(r['report']['preserved_unobserved_points'],0)
        self.assertTrue(np.any(np.all(new==far[0],axis=1)))
        self.assertEqual((destination/'.sessions/session_0/poses.txt').read_text(),'retained original trajectory')
        self.assertGreater(grid['image'].shape[1],40);self.assertEqual(grid['image'][0,0],0)
        self.assertEqual(base_fingerprint(self.base),before);self.assertEqual((self.maps/'active').resolve(),self.base.resolve())

    def test_successive_candidates_merge_previous_update_and_keep_active_original(self):
        service=MapService(self.maps,self.root/'data',demo=True);a=RefinementAgent(service,lambda:None)
        with patch('threading.Thread.start'):a.control({'action':'enable'})
        before=base_fingerprint(self.base)
        for batch in range(2):
            pts=np.vstack([self.floor,self.wall,self.wall+[-3,0,0]]) if batch else np.vstack([self.floor,self.wall])
            for i in range(3):
                n=batch*3+i;a.ingest(dict(protocol=1,frame_id='map',age=0,quality=[.02,.99],stamp=100+n*10,pose=[n*.3,0,.55,0,0,0,1],points=base64.b64encode(pts.astype('<f4').tobytes()).decode()))
            a.publish()
            self.assertEqual(a.row['frames'],0)
        self.assertEqual(len(a.row['revisions']),2,a.row)
        self.assertEqual(a.row['revisions'][1]['report']['base_map'],a.row['revisions'][0]['name'])
        self.assertEqual(a.row['base'],'original');self.assertEqual(a.row['total_frames'],6)
        self.assertEqual(base_fingerprint(self.base),before);self.assertEqual((self.maps/'active').resolve(),self.base.resolve());a.close()

    def test_internal_map_links_do_not_overwrite_copied_history(self):
        cache=self.base/'.nav_maps/nav_map_1';cache.mkdir(parents=True);(cache/'grid.pgm').write_bytes((self.base/'occ_grid.pgm').read_bytes())
        (self.base/'occ_grid_1.pgm').symlink_to('.nav_maps/nav_map_1/grid.pgm');(self.base/'occ_grid_1.yaml').write_text((self.base/'occ_grid.yaml').read_text())
        for filename in ['full_cloud.pcd','occ_grid.pgm','occ_grid.yaml']:
            source=self.base/filename;stored=self.base/('original-'+filename);source.rename(stored);source.symlink_to(stored.name)
        before=base_fingerprint(self.base)
        for i in range(3):self.frame(i)
        destination=self.maps/'linked-update';refine_map(self.base,self.session,destination,0,before)
        self.assertEqual((destination/'occ_grid_1.pgm').read_bytes(),(cache/'grid.pgm').read_bytes())
        for filename in ['full_cloud.pcd','occ_grid.pgm','occ_grid.yaml']:
            self.assertFalse((destination/filename).is_symlink());self.assertEqual((destination/('original-'+filename)).read_bytes(),(self.base/('original-'+filename)).read_bytes())
        self.assertEqual(base_fingerprint(self.base),before)

if __name__=='__main__':unittest.main()
