import sys,time,unittest
from pathlib import Path
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from operations import Operations

class OperationLifecycleTests(unittest.TestCase):
    def test_thread_start_failure_does_not_leave_running_task_or_consume_slots(self):
        ops=Operations();work=Mock()
        with patch('operations.threading.Thread.start',side_effect=RuntimeError('no thread resource')):
            for _ in range(5):
                with self.assertRaisesRegex(ValueError,'未能启动'):ops.submit('test',work)
        work.assert_not_called();jobs=ops.get()
        self.assertEqual(len(jobs),5);self.assertTrue(all(j['status']=='failed' and 'ended' in j for j in jobs))
        result=ops.submit('next',lambda progress:{'ok':True});deadline=time.monotonic()+1
        while time.monotonic()<deadline and ops.get(result['job_id'])['status']=='running':time.sleep(.01)
        self.assertEqual(ops.get(result['job_id'])['status'],'completed')

if __name__=='__main__':unittest.main()
