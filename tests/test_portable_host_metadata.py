"""Runner regression in a fresh local Python process; no product/NumPy import.

Audit hooks cannot be removed, so the guard check runs in an isolated process.
This harness is separate from the guarded product-test allowlist.
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

RUNNER=Path(__file__).resolve().parents[1]/'tools/check_portable.py'


class PortableHostMetadataTests(unittest.TestCase):
    def test_cached_metadata_works_after_guard_and_ver_is_not_a_test_exception(self):
        code=r'''
import importlib.util,json,sys
spec=importlib.util.spec_from_file_location('portable_metadata_test',sys.argv[1])
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
for name in ['numpy','cv2','m20_platform','m20.platform_core']:
    assert name not in sys.modules,name
prepared=runner.prepare_host_metadata()
assert prepared['stage']=='before_test_effect_guard'
assert not prepared['hostname_recorded']
for name in ['numpy','cv2','m20_platform','m20.platform_core']:
    assert name not in sys.modules,name
blocked,suppressed,self_test=runner.python_guard()
assert runner.verify_host_metadata_cache(prepared)['passed']
assert runner.platform.machine()==prepared['values']['machine']
assert blocked==[],blocked
for event,args in [
    ('subprocess.Popen',('cmd',['cmd','/c','ver'],None,None)),
    ('subprocess.Popen',('not-executed',['not-executed'],None,None)),
    ('socket.connect',(None,)),
]:
    try:sys.audit(event,*args)
    except RuntimeError:pass
    else:raise AssertionError('test effect was permitted: '+event)
assert blocked==['subprocess.Popen','subprocess.Popen','socket.connect'],blocked
print(json.dumps({'cache_verified':True,'blocked':blocked,'preparation_subprocesses':prepared['preparation_subprocesses']}))
'''
        result=subprocess.run([sys.executable,'-I','-B','-c',code,str(RUNNER)],capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        record=json.loads(result.stdout)
        self.assertTrue(record['cache_verified'])
        self.assertEqual(record['blocked'],['subprocess.Popen','subprocess.Popen','socket.connect'])

    def test_preparation_allows_only_exact_stdlib_windows_ver_queries(self):
        spec=importlib.util.spec_from_file_location('portable_metadata_unit',RUNNER)
        runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
        row=SimpleNamespace(system='Windows',release='fixture',version='fixture',machine='AMD64')
        def metadata(command):
            runner.subprocess.Popen(command,shell=True)
            return row
        for command in ['ver','command /c ver','cmd /c ver']:
            with patch.object(runner.sys,'platform','win32'),patch.object(runner.subprocess,'Popen') as process,patch.object(runner.platform,'uname',side_effect=lambda:metadata(command)),patch.object(runner.platform,'machine',return_value='AMD64'):
                result=runner.prepare_host_metadata()
                process.assert_called_once_with(command,shell=True)
                self.assertEqual(result['preparation_subprocesses'],['windows_version_builtin_ver'])
                self.assertIs(runner.subprocess.Popen,process)
        with patch.object(runner.sys,'platform','win32'),patch.object(runner.subprocess,'Popen') as process,patch.object(runner.platform,'uname',side_effect=lambda:metadata('cmd /c echo forbidden')):
            with self.assertRaisesRegex(RuntimeError,'Unexpected subprocess'):runner.prepare_host_metadata()
            process.assert_not_called();self.assertIs(runner.subprocess.Popen,process)


if __name__=='__main__':unittest.main()
