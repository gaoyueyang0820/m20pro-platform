import tempfile
import unittest
from pathlib import Path
from m20.platform_host import lock_workspace


class WorkspaceInstanceTests(unittest.TestCase):
    def test_second_backend_cannot_share_state_even_with_another_bind_address(self):
        with tempfile.TemporaryDirectory() as directory:
            first=lock_workspace(directory)
            try:
                with self.assertRaisesRegex(ValueError,'已经在运行'):lock_workspace(directory)
            finally:first.close()
            replacement=lock_workspace(directory);replacement.close()

    def test_isolated_demo_workspaces_remain_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            a=lock_workspace(Path(directory)/'live');b=lock_workspace(Path(directory)/'demo')
            a.close();b.close()


if __name__=='__main__':unittest.main()
