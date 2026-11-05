import sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.navigation_diagnostics import parse_navigation_monitor,read_navigation_diagnostics

def monitor(goal='YES',cancel='NO',speed='0.000',path=8):
    return f'''| Odom[OK] | Map[OK] | Goal[{goal}] | Cancel[{cancel}] | Mode: 自主导航(避障)
| Planner Status : Code = 1001 (正常输出速度) | Warning Code: 0
| Goal Info : 与目标距离 = 3.000 m
| Global Path: {path} waypoints
| Cmd Velocity : x_vel = {speed} | y_vel = 0.000 | yaw_vel = 0.000
| Parameters : 停障距离 = 0.60, 方向: 前进
'''

class NavigationDiagnosticTests(unittest.TestCase):
    def test_normal_status_with_repeated_zero_output_is_not_claimed_as_moving(self):
        r=parse_navigation_monitor(monitor()*8)
        self.assertEqual(r['consecutive_zero_output_records'],8);self.assertIn('零速度',r['message'])
        self.assertEqual(r['current']['reported_stop_distance_m'],.6)
    def test_current_idle_does_not_reuse_old_active_failure(self):
        r=parse_navigation_monitor(monitor()*8+monitor(goal='NO',cancel='YES',path=0))
        self.assertEqual(r['consecutive_zero_output_records'],0);self.assertIn('没有活动',r['message'])
        self.assertEqual(r['recent_active_zero_records'],8)
    def test_single_transition_zero_is_not_diagnosed_as_persistent_stagnation(self):
        r=parse_navigation_monitor(monitor(speed='.3')+monitor())
        self.assertEqual(r['consecutive_zero_output_records'],1);self.assertNotIn('连续',r['message'])
    def test_unknown_log_format_is_unavailable_not_healthy(self):
        self.assertFalse(parse_navigation_monitor('unrecognized log')['available'])
    def test_real_idle_placeholders_are_not_parsed_as_numbers(self):
        r=parse_navigation_monitor(monitor(goal='NO',cancel='YES',path=0).replace('3.000','--').replace('停障距离 = 0.60, 方向: 前进','--'))
        self.assertTrue(r['available']);self.assertIsNone(r['current']['goal_distance_m']);self.assertEqual(r['consecutive_zero_output_records'],0)
    def test_reader_only_reads_bounded_log_and_does_not_modify_it(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'2026_0101';folder.mkdir();file=folder/'planner.2026_0101.log';file.write_text(monitor()*5)
            before=file.read_bytes();r=read_navigation_diagnostics(Path(temp))
            self.assertTrue(r['read_only']);self.assertEqual(before,file.read_bytes())

if __name__=='__main__':unittest.main()
