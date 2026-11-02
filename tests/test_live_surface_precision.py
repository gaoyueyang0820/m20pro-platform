"""Positive measured-floor overrides with exact obstacles; no live I/O."""
import copy
import math
import unittest
from unittest.mock import patch
import numpy as np
from live_traversability import LiveTraversability
from start_pose_recovery import recover_start
from test_live_surface_localization import door_world,planner,packet,feed


def legacy_visible(scan,resolution):
    floor=set(map(tuple,scan['floor']))&set(map(tuple,scan['measured_floor']))
    obstacles=set(map(tuple,scan['obstacles']));floor-=obstacles;visible=set()
    for x,y in floor:
        if math.hypot(x,y)*resolution>2.2:continue
        n=max(abs(x),abs(y))
        if not any((round(x*i/n),round(y*i/n)) in obstacles for i in range(1,max(1,n))):visible.add((x,y))
    return visible


class LiveSurfacePrecisionTests(unittest.TestCase):
    def add_small_box(self,scan,q=(4,2),box=(.888,.488,.892,.492)):
        q=list(q);scan['obstacles'].append(q);scan['obstacle_bounds'].append(list(box))
        scan['floor']=[p for p in scan['floor'] if p!=q]
        if q not in scan['measured_floor']:scan['measured_floor'].append(q)

    def test_measured_free_part_of_same_coarse_obstacle_cell_can_be_confirmed(self):
        old_grid=door_world();target=planner(old_grid).cell(3.55,4.35);old_grid[target[1],target[0]]=0
        truth=planner(door_world(True));results=[]
        for legacy in [True,False]:
            p=planner(old_grid,True);before=p.base_free.copy()
            for i in range(3):
                s,l=packet(truth,stamp=100+i*.2);self.add_small_box(s)
                if legacy:
                    with patch.object(LiveTraversability,'_visible_floor',staticmethod(legacy_visible)):p.update_live_surface(s,l,now=100.08+i*.2)
                else:p.update_live_surface(s,l,now=100.08+i*.2)
            results.append(bool(p.free[target[1],target[0]]));np.testing.assert_array_equal(p.base_free,before)
            # The real obstacle's own fine map cell always stays occupied.
            actual=p.cell(2.8+.89,4.+.49);self.assertFalse(p.free[actual[1],actual[0]])
        self.assertEqual(results,[False,True])

    def test_precise_shadow_still_blocks_supported_points_behind_real_obstacle(self):
        p=planner(door_world(),True);truth=planner(door_world(True));target=p.cell(4.,4.)
        for i in range(4):
            s,l=packet(truth,stamp=100+i*.2);self.add_small_box(s,(4,0),(.78,-.06,.82,.06))
            p.update_live_surface(s,l,now=100.08+i*.2)
        self.assertFalse(p.free[target[1],target[0]])

    def test_absent_or_malformed_bounds_keep_the_conservative_cell_filter(self):
        s,l=packet(planner(door_world(True)));self.add_small_box(s)
        self.assertIn((4,2),LiveTraversability._visible_floor(s,.2))
        for bounds in [[],[[1,2,3]],[[float('nan'),0,1,1]]]:
            bad=dict(s,obstacle_bounds=bounds)
            self.assertNotIn((4,2),LiveTraversability._visible_floor(bad,.2))

    def test_no_ground_return_cannot_be_created_by_exact_box_visibility(self):
        p=planner(door_world(),True);truth=planner(door_world(True))
        for i in range(4):
            s,l=packet(truth,stamp=100+i*.2);s['floor']=[];s['measured_floor']=[]
            p.update_live_surface(s,l,now=100.08+i*.2)
        self.assertEqual(p.live_surface.free_cells,set())

    def test_unobserved_initial_body_overlap_reports_seed_gap_without_clearing_wall(self):
        p=planner(door_world(),True);truth=planner(door_world(True));before=p.base_free.copy();start=[3.6,4.,0.]
        for i in range(4):
            s,l=packet(truth,actual=start,stamp=100+i*.2)
            for key in ['floor','measured_floor']:s[key]=[q for q in s[key] if math.hypot(*q)*.2>.55]
            p.update_live_surface(s,l,now=100.08+i*.2)
        result=recover_start(p,start,[2.,4.],0.,s,l,standing=True)
        self.assertFalse(result['ok']);self.assertEqual(result['classification'],'map_surface_disagreement')
        self.assertGreater(result['overlap_diagnostics']['unobserved_body_cells'],0)
        self.assertTrue(result['overlap_diagnostics']['standing_is_not_map_clearing_evidence'])
        self.assertEqual(result['reason_code'],'unobserved_body_overlap')
        np.testing.assert_array_equal(p.base_free,before)

    def test_previous_real_floor_can_carry_under_body_but_live_contact_still_rejects(self):
        p=planner(door_world(),True);truth=planner(door_world(True));feed(p,truth)
        for i,x in enumerate([3.,3.2,3.4,3.6]):
            s,l=packet(truth,actual=(x,4.,0.),stamp=101+i*.2)
            for key in ['floor','measured_floor']:s[key]=[q for q in s[key] if math.hypot(*q)*.2>.55]
            p.update_live_surface(s,l,now=101.08+i*.2)
        self.assertTrue(p.pose_fits(3.6,4.,0.))
        result=recover_start(p,[3.6,4.,0.],[5.2,4.],0.,s,l,standing=True)
        self.assertTrue(result['ok'],result)
        contact=copy.deepcopy(s);self.add_small_box(contact,(1,0),(.19,-.02,.21,.02))
        result=recover_start(p,[3.6,4.,0.],[5.2,4.],0.,contact,l,standing=True)
        self.assertFalse(result['ok']);self.assertEqual(result['classification'],'live_body_collision')


if __name__=='__main__':unittest.main()
