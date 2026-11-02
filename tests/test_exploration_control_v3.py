"""Continuous exploration integration; ideal kinematics, no hardware I/O."""
import unittest
from tools.benchmark_exploration_control_v3 import run


class ExplorationControlTests(unittest.TestCase):
    def test_two_occluded_rooms_with_continuous_commands(self):
        result=run(steps=650)
        summary={k:v for k,v in result.items() if k!='trace'}
        self.assertIsNotNone(result['first_door_crossed'],str(summary))
        self.assertFalse(result['collision'],str(summary))
        self.assertGreaterEqual(result['coverage'],.98,str(summary))
        self.assertLess(result['distance_m'],45,str(summary))
        self.assertTrue(any(abs(p['axes'][2])>.1 or abs(p['axes'][1])>.1 for p in result['trace']))


if __name__=='__main__':unittest.main()
