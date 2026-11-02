import importlib.util,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('delivery_builder',ROOT/'tools/build_delivery.py')
builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)

class DeliveryBoundaryTests(unittest.TestCase):
    def test_real_replays_and_runtime_profiles_are_not_source_delivery(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for name in ['main.py','tests/fixtures/runtime_replay/scan.json','tests/fixtures/runtime_replay/guard.cjs','data/connections.json','tests/test_fake.py']:
                p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('{}')
            names={p.relative_to(root).as_posix() for p in builder.source_files(root)}
            self.assertNotIn('data/connections.json',names);self.assertNotIn('tests/fixtures/runtime_replay/scan.json',names)
            self.assertIn('tests/fixtures/runtime_replay/guard.cjs',names);self.assertIn('tests/test_fake.py',names)

if __name__=='__main__':unittest.main()
