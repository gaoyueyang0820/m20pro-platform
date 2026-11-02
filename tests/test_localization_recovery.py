"""Offline geometry checks; no robot, server, files or external dependencies."""
import math
import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from localization_recovery import match_pose, verify_pose, saved_slam_pose, verify_transform


def structure():
    height=np.linspace(-.2,1.35,19)
    def wall(xs,ys):
        return np.array([[x,y,z] for x,y in zip(xs,ys) for z in height])
    a=np.linspace(-2.7,3.7,90)
    b=np.linspace(-2.4,2.1,70)
    return np.concatenate([
        wall(np.full_like(b,-2.7),b),wall(a,np.full_like(a,2.1)),
        wall(np.linspace(.7,1.6,35),np.linspace(-1.2,-.5,35)),
        wall(np.full(30,3.7),np.linspace(-.3,1.2,30)),
    ])


def transformed(points,pose):
    c,s=math.cos(pose[3]),math.sin(pose[3])
    return points@np.array([[c,-s,0],[s,c,0],[0,0,1]]).T+pose[:3]


def rigid(roll=.1,pitch=-.08,yaw=.4,position=(.3,-.2,.04)):
    c,s=math.cos,math.sin
    rx=np.array([[1,0,0],[0,c(roll),-s(roll)],[0,s(roll),c(roll)]])
    ry=np.array([[c(pitch),0,s(pitch)],[0,1,0],[-s(pitch),0,c(pitch)]])
    rz=np.array([[c(yaw),-s(yaw),0],[s(yaw),c(yaw),0],[0,0,1]])
    matrix=np.eye(4);matrix[:3,:3]=rz@ry@rx;matrix[:3,3]=position
    return matrix


def saved_fixture():
    context=dict(workflow='extension',base='base',grid_id=0,anchor=dict(x=0,y=0,z=0,yaw=0))
    fusion=dict(base_map='base',map='base-r2',source_map='capture-123',created=114,
                transform=rigid().tolist(),structural_alignment=dict(
                    structural_overlap=.96,rmse_m=.064,effective_frames=3,
                    correction_m=.04,correction_yaw_rad=.004,known_free_conflict_ratio=.01,
                    decision=dict(allowed=True,requires_review=False)))
    build=dict(phase='saved',in_progress=False,result_maps=['base-r2'],
        mapping=dict(context=copy.deepcopy(context),name='capture',started=100,state='command_finished'),
        stop_mapping=dict(context=copy.deepcopy(context),started=110,ended=115,state='command_finished',returncode=0,
                          result_maps=['base-r2'],fusion=fusion))
    sensors=dict(slam_odom=dict(frame_id='map',child_frame_id='',received_at=110.1,stamp=110,
                               PosX=.4,PosY=-.2,PosZ=.01,quaternion=[0,0,0,1]),
                 slam_pose_proof=dict(verified=True,method='paired_base_link_and_map_clouds',stamp=103))
    return build,sensors


class LocalizationRecoveryTests(unittest.TestCase):
    def test_recovers_position_and_large_heading_from_local_prior(self):
        body=structure();pose=np.array([.35,-.25,.04,2.21]);reference=transformed(body,pose)
        result=match_pose(body,reference,dict(x=0,y=0,z=0),stamp=100)
        actual=result['pose']
        self.assertLess(math.hypot(actual['PosX']-pose[0],actual['PosY']-pose[1]),.07)
        self.assertLess(abs(math.atan2(math.sin(actual['Yaw']-pose[3]),math.cos(actual['Yaw']-pose[3]))),.025)
        self.assertGreater(result['quality']['overlap'],.98)
        self.assertTrue(result['requires_independent_validation'])

    def test_new_scan_verifies_but_same_or_older_stamp_cannot(self):
        body=structure();pose=np.array([.2,.1,.03,-.39]);reference=transformed(body,pose)
        result=match_pose(body,reference,[0,0,0],stamp=123)
        fresh=body+np.random.default_rng(4).normal(scale=.006,size=body.shape)
        verified=verify_pose(result,fresh,reference,stamp=123.1)
        self.assertTrue(verified['verified']);self.assertGreater(verified['quality']['overlap'],.98)
        for stamp in [123,122]:
            with self.assertRaisesRegex(ValueError,'新帧'):verify_pose(result,fresh,reference,stamp=stamp)
        with self.assertRaisesRegex(ValueError,'没有确认'):verify_pose(result,fresh+[1.1,.4,0],reference,stamp=124)

    def test_symmetric_room_does_not_choose_arbitrary_heading(self):
        axis=np.linspace(-2.5,2.5,65);height=np.linspace(-.2,1.35,16)
        body=np.array([[x,y,z] for z in height for a in axis
                       for x,y in [(-2.5,a),(2.5,a),(a,-2.5),(a,2.5)]])
        with self.assertRaisesRegex(ValueError,'多个'):match_pose(body,body,[0,0,0],stamp=1)

    def test_wrong_local_map_is_not_accepted(self):
        rng=np.random.default_rng(17)
        reference=rng.uniform([-5,-5,-.3],[5,5,1.6],size=(2000,3))
        with self.assertRaises(ValueError):match_pose(structure(),reference,[0,0,0],stamp=1)

    def test_dominant_floor_or_single_wall_cannot_establish_pose(self):
        x,y=np.meshgrid(np.linspace(-3,3,70),np.linspace(-3,3,70))
        floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55)])
        wall=np.column_stack([np.full(x.size,2),x.ravel(),y.ravel()/4])
        for body in [floor,wall]:
            with self.assertRaisesRegex(ValueError,'结构'):match_pose(body,structure(),[0,0,0],stamp=1)
        with self.assertRaisesRegex(ValueError,'base_link'):match_pose(structure(),structure(),[0,0,0],stamp=1,frame_id='map')


class SavedSlamRecoveryTests(unittest.TestCase):
    def test_saved_pose_composes_full_transform_and_preserves_tilt(self):
        build,sensors=saved_fixture();result=saved_slam_pose(build,sensors,'base')
        expected=rigid();expected[:3,3]+=expected[:3,:3]@np.array([.4,-.2,.01])
        np.testing.assert_allclose(result['transform'],expected)
        self.assertEqual(result['source_map'],'capture-123')
        self.assertTrue(result['requires_independent_validation'])
        self.assertGreater(abs(result['transform'][2][0]),.05)

    def test_wrong_map_session_and_structural_failure_are_rejected(self):
        changes=[lambda b,s:b['mapping']['context'].update(base='other'),
                 lambda b,s:b['stop_mapping']['context']['anchor'].update(x=1),
                 lambda b,s:b['stop_mapping']['fusion'].update(source_map='other-123'),
                 lambda b,s:b['stop_mapping']['fusion']['structural_alignment']['decision'].update(allowed=False),
                 lambda b,s:b['stop_mapping']['fusion']['structural_alignment'].update(structural_overlap=.3),
                 lambda b,s:b.update(phase='collecting',in_progress=True)]
        for change in changes:
            with self.subTest(change=change):
                b,s=saved_fixture();change(b,s)
                with self.assertRaises(ValueError):saved_slam_pose(b,s,'base')
        b,s=saved_fixture()
        with self.assertRaises(ValueError):saved_slam_pose(b,s,'different-active-map')

    def test_old_cached_pose_and_old_coordinate_proof_are_rejected(self):
        changes=[lambda b,s:s['slam_odom'].update(received_at=99,stamp=99),
                 lambda b,s:s['slam_odom'].update(received_at=107,stamp=107),
                 lambda b,s:s['slam_odom'].update(received_at=116,stamp=116),
                 lambda b,s:s['slam_pose_proof'].update(stamp=90),
                 lambda b,s:s['slam_odom'].update(child_frame_id='other_frame'),
                 lambda b,s:b['stop_mapping']['fusion'].update(created=116)]
        for change in changes:
            with self.subTest(change=change):
                b,s=saved_fixture();change(b,s)
                with self.assertRaises(ValueError):saved_slam_pose(b,s,'base')

    def test_full_3d_validation_succeeds_while_yaw_only_pose_fails(self):
        # Horizontal shelves make roll/pitch observable too; a dense vertical
        # wall alone can hide small attitude errors through nearest-z matches.
        x,y=np.meshgrid(np.linspace(-1.8,1.8,48),np.linspace(-1.8,1.8,48))
        shelves=np.concatenate([np.column_stack([x.ravel(),y.ravel(),np.full(x.size,z)]) for z in [.35,.9]])
        body=np.concatenate([structure(),shelves]);matrix=rigid();reference=body@matrix[:3,:3].T+matrix[:3,3]
        result=verify_transform(matrix,body,reference,stamp=120,after_stamp=110)
        self.assertGreater(result['quality']['overlap'],.98)
        np.testing.assert_allclose(result['validation_transform'],matrix)
        pose=result['initialization_pose'];yaw_only=rigid(0,0,pose['Yaw'],matrix[:3,3])
        with self.assertRaisesRegex(ValueError,'没有确认'):verify_transform(yaw_only,body,reference,stamp=120,after_stamp=110)
        with self.assertRaisesRegex(ValueError,'新帧'):verify_transform(matrix,body,reference,stamp=110,after_stamp=110)

    def test_invalid_rigid_matrix_cannot_be_used(self):
        bad=np.eye(4);bad[0,0]=2
        with self.assertRaises(ValueError):verify_transform(bad,structure(),structure(),stamp=120,after_stamp=110)


if __name__=='__main__':unittest.main()
