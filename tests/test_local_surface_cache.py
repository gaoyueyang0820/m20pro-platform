"""Exact geometry memoization and obstacle-driven exploration, synthetic only."""
import copy
import json
import math
import threading
import time
import unittest
from unittest.mock import patch

from autonomy_core import Explorer,disk
from local_trajectory import LocalSurface,TrajectoryFollower,exploration_step


def scene(stamp=1.):
    floor=[list(p) for p in disk(3)]
    return dict(pose=[0.,0.,0.],stamp=stamp,geometry_version=2,floor=floor,measured_floor=copy.deepcopy(floor),
                obstacles=[[5,0],[3,4]],obstacle_bounds=[[.99,-.01,1.01,.01],[.59,.79,.61,.81]],_speed_mps=0.,_velocity_body=[0.,0.])


class LocalSurfaceCacheTests(unittest.TestCase):
    def test_repeated_queries_reuse_exact_result_and_failure_cause(self):
        surface=LocalSurface.cached(scene());surface.cache.clear();surface.sweeps.clear()
        with patch.object(surface,'_sweep_uncached',wraps=surface._sweep_uncached) as sweep:
            self.assertFalse(surface.sweep_valid(.6,0));self.assertEqual(surface.last_failure,'obstacle')
            self.assertTrue(surface.sweep_valid(-.4,0));self.assertIsNone(surface.last_failure)
            self.assertFalse(surface.sweep_valid(.6,0));self.assertEqual(surface.last_failure,'obstacle')
            self.assertEqual(sweep.call_count,2)
        surface.cache.clear()
        with patch.object(surface,'_valid_uncached',wraps=surface._valid_uncached) as valid:
            self.assertTrue(surface.valid(0,0,0));self.assertTrue(surface.valid(0,0,0));self.assertEqual(valid.call_count,1)

    def test_cached_and_fresh_geometry_match_for_rotations_translations_and_padding(self):
        data=scene();cached=LocalSurface.cached(data);fresh=LocalSurface(data)
        for x in [-.8,-.3,0.,.2,.4,.7,1.]:
            for y in [-.6,-.2,0.,.2,.6]:
                for yaw in [-math.pi/2,-.7,0.,.23,math.pi/2]:
                    for padding in [(0.,0.),(.031,.017)]:
                        expected=fresh.valid(x,y,yaw,padding=padding);reason=fresh.last_failure
                        self.assertEqual(cached.valid(x,y,yaw,padding=padding),expected)
                        self.assertEqual(cached.last_failure,reason)
        for dx,dy in [(0.,0.),(.4,0.),(-.4,0.),(0.,.4),(0.,-.4),(.3,.3)]:
            for turn in [-.7,0.,.7]:
                expected=fresh.sweep_valid(dx,dy,turn);reason=fresh.last_failure
                self.assertEqual(cached.sweep_valid(dx,dy,turn),expected);self.assertEqual(cached.last_failure,reason)

    def test_same_stamp_mutated_points_bounds_and_support_cannot_reuse_old_permission(self):
        data=scene();first=LocalSurface.cached(data);self.assertTrue(first.valid(0,0,0))
        data['obstacle_bounds'][0][:]=[.39,-.01,.41,.01];data['obstacles'][0][:]=[2,0]
        contact=LocalSurface.cached(data);self.assertIsNot(contact,first);self.assertFalse(contact.valid(0,0,0))
        data=scene();first=LocalSurface.cached(data);self.assertTrue(first.valid(-.5,0,0))
        for key in ['floor','measured_floor']:data[key][:]=[p for p in data[key] if p[0]>=0]
        unknown=LocalSurface.cached(data);self.assertIsNot(unknown,first);self.assertFalse(unknown.valid(-.5,0,0));self.assertEqual(unknown.last_failure,'support')

    def test_geometry_reuse_never_changes_frame_expiry_or_serializable_scan(self):
        data=scene();data['_valid_until']=time.monotonic()-.01;before=json.dumps(data,sort_keys=True)
        first=LocalSurface.cached(data);later=copy.deepcopy(data);later.update(stamp=2,pose=[3.,1.,.4],_speed_mps=.5)
        self.assertIs(LocalSurface.cached(later),first)
        self.assertEqual(json.dumps(data,sort_keys=True),before);self.assertLess(data['_valid_until'],time.monotonic())
        shifted=copy.deepcopy(data)
        shifted['obstacle_bounds']=[[a-.4,b,c-.4,d] for a,b,c,d in shifted['obstacle_bounds']]
        self.assertIsNot(LocalSurface.cached(shifted),first)

    def test_query_failure_state_is_not_shared_across_worker_threads(self):
        main=LocalSurface.cached(scene());other=[]
        def work():other.append(LocalSurface.cached(scene()))
        thread=threading.Thread(target=work);thread.start();thread.join(1)
        self.assertFalse(thread.is_alive());self.assertEqual(len(other),1);self.assertIsNot(main,other[0])


class BlockedExplorationTests(unittest.TestCase):
    def blocked(self,stamp=1.):
        data=dict(pose=[0.,0.,0.],stamp=stamp,floor=[list(p) for p in disk(3)],obstacles=[[3,y] for y in range(-8,9)],_speed_mps=0.,_velocity_body=[0.,0.])
        explorer=Explorer(4);explorer.ingest(data);explorer.path=[(i,0) for i in range(11)];explorer.goal=(10,0)
        return explorer,TrajectoryFollower(),data

    def test_stopped_controller_requests_other_direction_instead_of_repeating_zero(self):
        e,f,data=self.blocked();result=exploration_step(e,f,data)
        self.assertEqual(result['state'],'blocked');self.assertTrue(result['needs_recovery']);self.assertEqual(result['axes'],[0.,0.,0.])
        self.assertTrue(e.request_recovery((.35,0,0)))
        self.assertEqual(e.command_axes(),(0.,0.,0.),'changing direction still waits for a later stopped feedback frame')
        later=dict(data,stamp=2.);e.ingest(later);axes=e.command_axes()
        self.assertTrue(any(axes),e.motion_hint);self.assertTrue(axes[0]<0 or abs(axes[1])>.1 or abs(axes[2])>.1)

    def test_cached_or_expired_frames_cannot_start_a_new_escape_or_repair(self):
        for cached in [False,True]:
            e,f,data=self.blocked();data['_valid_until']=time.monotonic()-.01 if not cached else time.monotonic()+1
            result=exploration_step(e,f,data,new_scan=not cached)
            self.assertFalse(result.get('needs_recovery',False))
        e,f,data=self.blocked();e.path=[(30,30),(31,30)];data['_valid_until']=time.monotonic()-.01
        with patch.object(e,'plan',side_effect=AssertionError('expired observation cannot rebuild path')):
            result=exploration_step(e,f,data)
        self.assertEqual(result['axes'],[0.,0.,0.]);self.assertFalse(result.get('needs_recovery',False))


if __name__=='__main__':unittest.main()
