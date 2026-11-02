"""Offline geometry benchmarks. No robot I/O, no claim of sensor calibration."""
import json,math,sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_registration import fit_structure,structure_error,independently_validated_correction
from map_visibility import RangeVisibility,confirmed_surface_removals
from map_refinement import refine_map,base_fingerprint,aligned_observations
from map_fusion import write_pcd,read_pcd,read_grid,refine_slam_structure,pose_matrix,transform


def corner():
    axis=np.arange(-2,2.01,.05);z=np.arange(.2,1.21,.1)
    y,h=np.meshgrid(axis,z)
    return np.vstack([np.column_stack([np.full(y.size,2.),y.ravel(),h.ravel()]),
                      np.column_stack([y.ravel(),np.full(y.size,2.),h.ravel()])]).astype(np.float32)


def world_frames(point,occluded=False,trusted=True):
    frames=[]
    for i,y in enumerate([0,.4,.8,-.4,-.8]):
        origin=np.array([0.,y,.55]);behind=origin+2*(point-origin)
        points=[behind]
        if occluded:points.append(origin+.5*(point-origin))
        frames.append(dict(points=np.array(points),stamp=i*10,pose=[0,y,.55,0,0,0,1],
                           sensor_origin_world=origin,sensor_origin_trusted=trusted))
    return frames


class VisibilityTests(unittest.TestCase):
    def test_removed_surface_has_measured_behind_returns_not_missing_returns(self):
        point=np.array([2.,0.,.7]);frames=world_frames(point)
        removed,r=confirmed_surface_removals(np.array([point]),frames)
        self.assertEqual(removed.tolist(),[True]);self.assertEqual(r['votes']['free_ray'],5)
        removed,r=confirmed_surface_removals(np.array([point]),[dict(f,points=np.empty((0,3))) for f in frames])
        self.assertFalse(removed.any());self.assertEqual(r['votes']['unknown_or_occluded'],5)

    def test_occlusion_and_unseen_regions_are_preserved(self):
        point=np.array([2.,0.,.7]);unseen=np.array([0.,-3.,.7])
        removed,r=confirmed_surface_removals(np.array([point,unseen]),world_frames(point,occluded=True))
        self.assertFalse(removed.any());self.assertEqual(r['votes']['free_ray'],0)

    def test_body_pose_is_never_silently_used_as_sensor_origin(self):
        point=np.array([2.,0.,.7]);removed,r=confirmed_surface_removals(np.array([point]),world_frames(point,trusted=False))
        self.assertFalse(removed.any());self.assertEqual(r['state'],'no_calibrated_single_sensor_origin')
        frames=world_frames(point)
        for f in frames:f.pop('sensor_origin_world')
        removed,r=confirmed_surface_removals(np.array([point]),frames)
        self.assertFalse(removed.any())

    def test_reappearing_surface_and_insufficient_view_span_do_not_clear(self):
        point=np.array([2.,0.,.7]);frames=world_frames(point);frames[-1]['points']=point.reshape(1,3)
        self.assertFalse(confirmed_surface_removals(point.reshape(1,3),frames)[0].any())
        self.assertFalse(confirmed_surface_removals(point.reshape(1,3),world_frames(point)[:4])[0].any())

    def test_coarse_occlusion_edge_vetoes_fine_bin_clear(self):
        q=np.array([[2.,0.,.7]]);origin=np.array([0.,0.,.55]);far=origin+2*(q[0]-origin)
        edge=np.array([1.,.006,.625])
        self.assertEqual(RangeVisibility(np.array([far,edge]),origin).classify(q).tolist(),[0])

    def test_ray_surface_removal_does_not_invent_ground_or_clear_grid(self):
        point=np.array([2.,0.,.7]);removed,report=confirmed_surface_removals(point.reshape(1,3),world_frames(point))
        self.assertTrue(removed[0]);self.assertFalse(report['occupancy_cleared_by_rays'])


class StructureRefinementTests(unittest.TestCase):
    def test_minor_consistent_bias_is_corrected_instead_of_thickening_walls(self):
        old=corner();source=old+[.08,-.06,0]
        T,report=independently_validated_correction([(i,source,old) for i in [0,2,4]])
        before=structure_error(source,old)['rmse_m'];after=structure_error(source,old,T)['rmse_m']
        self.assertTrue(report['applied'],report);self.assertLess(after,.01);self.assertLess(after,before*.2)
        self.assertEqual(len(report['independent_frames']),2)

    def test_single_wall_does_not_invent_unobservable_translation(self):
        old=corner();old=old[np.isclose(old[:,0],2.)]
        T,report=fit_structure(old+[.08,-.06,0],old)
        self.assertFalse(report['applied']);self.assertEqual(report['status'],'degenerate_structure');np.testing.assert_allclose(T,np.eye(4))

    def test_localization_jump_is_not_repaired_by_arbitrary_large_map_shift(self):
        old=corner();T,report=fit_structure(old+[.6,.6,0],old)
        self.assertFalse(report['applied']);np.testing.assert_allclose(T,np.eye(4))

    def test_middle_frame_cannot_overrule_independent_frames(self):
        old=corner();T,report=independently_validated_correction([(0,old,old),(2,old+[.08,-.06,0],old),(4,old,old)])
        self.assertFalse(report['applied']);self.assertEqual(report['status'],'independent_validation_failed');np.testing.assert_allclose(T,np.eye(4))

    def test_new_object_does_not_pull_already_aligned_wall_anchors(self):
        old=corner();yy,zz=np.meshgrid(np.arange(-.3,.31,.05),np.arange(.2,1.2,.1));obj=np.column_stack([np.full(yy.size,1.),yy.ravel(),zz.ravel()])
        T,report=fit_structure(np.vstack([old,obj]),old)
        self.assertFalse(report['applied']);np.testing.assert_allclose(T,np.eye(4))

    def test_complete_passive_update_preserves_old_map_and_raw_observations(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base';base.mkdir();session=root/'observations';session.mkdir()
            axis=np.arange(-3,3.01,.1);x,y=np.meshgrid(axis,axis);floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
            old=np.column_stack([np.vstack([floor,corner()]),np.full(len(floor)+len(corner()),31.)]).astype(np.float32)
            # Far geometry outside all observations must retain exact coordinates/intensity.
            old=np.vstack([old,[9.,9.,1.,77.]])
            write_pcd(base/'full_cloud.pcd',old)
            (base/'occ_grid.pgm').write_bytes(b'P5\n100 100\n255\n'+bytes([254])*10000)
            (base/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.1\norigin: [-5,-5,0]\n')
            points=np.vstack([floor,corner()])+[.08,-.06,0]
            for i in range(5):np.savez(session/('frame-%06d.npz'%i),points=points,pose=[-i*.3+.08,-.06,.55,0,0,0,1],stamp=i*10)
            before={p.name:p.read_bytes() for p in session.iterdir()};fingerprint=base_fingerprint(base)
            result=refine_map(base,session,root/'updated',0,fingerprint)
            self.assertTrue(result['report']['structural_correction']['applied']);self.assertFalse(result['activated'])
            self.assertEqual(base_fingerprint(base),fingerprint)
            self.assertEqual({p.name:p.read_bytes() for p in session.iterdir()},before)
            cloud=read_pcd(root/'updated/full_cloud.pcd');self.assertTrue(np.any(np.all(cloud==[9,9,1,77],axis=1)))
            self.assertEqual(result['report']['old_surface_visibility']['state'],'no_calibrated_single_sensor_origin')
            wall=cloud[(cloud[:,2]>.2)&(cloud[:,0]>1.7)&(abs(cloud[:,1])<1.7)]
            self.assertLess(np.quantile(abs(wall[:,0]-2),.95),.015)

    def test_complete_update_removes_visible_surface_but_keeps_unobserved_grid_support(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base';base.mkdir();session=root/'observations';session.mkdir()
            axis=np.arange(-3,3.01,.1);x,y=np.meshgrid(axis,axis);floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
            ghost=np.array([1.,0.,.8]);floor=floor[np.linalg.norm(floor[:,:2]-ghost[:2],axis=1)>.35]
            points=np.vstack([floor,corner()]);old=np.vstack([points,ghost])
            write_pcd(base/'full_cloud.pcd',old)
            image=np.full((100,100),254,np.uint8);image[49,60]=0
            (base/'occ_grid.pgm').write_bytes(b'P5\n100 100\n255\n'+image.tobytes())
            (base/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.1\norigin: [-5,-5,0]\n')
            for i in range(5):
                origin=np.array([-i*.4,0,.55]);far=origin+(ghost-origin)*(2-origin[0])/(ghost[0]-origin[0])
                np.savez(session/('frame-%06d.npz'%i),points=np.vstack([points,far]),pose=[origin[0],0,.55,0,0,0,1],stamp=i*10,
                         sensor_origin_world=origin,sensor_origin_trusted=True)
            result=refine_map(base,session,root/'updated',0,base_fingerprint(base))
            self.assertGreater(result['report']['old_surface_visibility']['removed_points'],0)
            cloud=read_pcd(root/'updated/full_cloud.pcd')
            self.assertFalse(np.any(np.linalg.norm(cloud[:,:3]-ghost,axis=1)<.03))
            self.assertEqual(read_grid(root/'updated')['image'][49,60],0)

    def test_same_structural_correction_applies_to_original_map_extension(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);clouds=root/'.sessions/session_0/lidar_cloud';clouds.mkdir(parents=True)
            old=corner();bias=np.array([.08,-.06,0]);poses=[]
            for i in range(5):
                pose=[-i*.3,0,.55,0,0,0,1];poses.append([i*10]+pose)
                write_pcd(clouds/('%d.pcd'%i),transform(old+bias,np.linalg.inv(pose_matrix(pose))))
            grid=dict(image=np.full((100,100),254,np.uint8),origin=[-5,-5,0],resolution=.1)
            T,report=refine_slam_structure(old,root,np.array(poses),np.eye(4),grid)
            self.assertTrue(report['applied']);self.assertLess(structure_error(old+bias,old,T)['rmse_m'],.01)


if __name__=='__main__':unittest.main()
