"""Offline dispatch tests; never load ROS or contact a robot."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch,MagicMock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge.sensor_agent import SensorAgent

class SensorReaderTests(unittest.TestCase):
    def test_reader_exposes_observation_only(self):
        sensor=MagicMock()
        with patch('bridge.sensor_agent.RosSensors',return_value=sensor):reader=SensorAgent()
        for path in ['/velocity','/cmd','/maps/build','/service','/joints/arm']:
            with self.assertRaisesRegex(ValueError,'只读'):reader.dispatch(path,{})
        self.assertEqual(sensor.method_calls,[])
        reader.dispatch('/sensors/status',None);sensor.status.assert_called_once()
        reader.dispatch('/sensors/cloud',{'topic':'/LIDAR/POINTS'});sensor.preview.assert_called_once_with('/LIDAR/POINTS')
        reader.dispatch('/sensors/autonomy',{'mapping':True});sensor.autonomy.assert_called_once_with(True)
        reader.close();sensor.close.assert_called_once()

if __name__=='__main__':unittest.main()
