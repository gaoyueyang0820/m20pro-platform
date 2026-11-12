#!/usr/bin/env python3
"""Portable, allowlisted source checks; never opens a robot or local API.

Dependencies are installed separately. This command explicitly builds a fresh
C++ library in a temporary source snapshot, then runs guarded Python/Node tests.
No private data, model weights, recorded JSON fixtures or existing native binary
is copied into that snapshot. Language guards are NOT an OS security sandbox.
"""
import argparse
import ast
from concurrent.futures import ThreadPoolExecutor,as_completed
from datetime import datetime,timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
MARKER='PORTABLE_WORKER_RESULT='
GROUPS={
    'algorithms':['test_autonomy_recovery','test_exploration_frontiers','test_exploration_completion','test_exploration_control_v3',
                  'test_local_trajectory','test_local_surface_cache','test_exploration_flow','test_follow_control','test_exploration_view',
                  'test_recovery_issuance','test_precise_clearance.PreciseClearanceTests',
                  'test_control_clock','test_terrain_profiles','test_pose_monitor','test_greeting','test_greeting_cli'],
    'navigation':['test_navigation_planner','test_navigation_planning_v3','test_live_surface_localization',
                  'test_localization_recovery','test_navigation_intents','test_goal_approach','test_goal_approach_review','test_body_projection_start','test_navigation_speckles','test_navigation_motion_alignment','test_planning_snapshot'],
    'sensing':['test_sensor_cache','test_sensor_reader','test_latest_observation','test_ground_recovery','test_sensing_equivalence','test_manual_turn_scan_frames','test_live_surface_precision'],
    'maps':['test_map_change_policy','test_map_fusion','test_map_update_v3','test_map_catalog','test_map_catalog_review',
            'test_workflows.HubTests.test_navigation_sync_reuses_exact_grid_across_robot_addresses',
            'test_map_learning.MapLearningTests.test_recorder_quality_and_duplicate_stamp_checks',
            'test_mapping_save_outcomes.SyntheticSavedObservationTests'],
    'lifecycle':['test_mapping_request_identity','test_operations_lifecycle','test_delivery_boundary','test_navigation_preview_identity','test_greeting_runtime','test_shared_listener.SharedStartupTests','test_map_management_hub','test_map_management_review','test_connection_recovery','test_sdk_setup_review',
                 'test_navigation_lifecycle.NavigationLifecycleTests.test_native_route_keeps_running_without_desktop_location_and_advances_on_arrival',
                 'test_navigation_lifecycle.NavigationLifecycleTests.test_new_idle_report_and_stationary_feedback_reconcile_a_lost_stop_ack',
                 'test_navigation_lifecycle.NavigationLifecycleTests.test_stop_can_be_retried_after_receipt_timeout',
                 'test_navigation_lifecycle.NavigationLifecycleTests.test_original_route_waits_for_station_count_before_next_goal',
                 'test_autonomy','test_point_navigation_runtime','test_point_navigation_recovery','test_platform_route_dispatch','test_map_activation_localization','test_gos_patrol',
                 'test_autonomy_observation_runtime.AutonomyObservationRuntimeTests.test_ground_occlusion_survives_long_wait_and_resumes_same_task',
                 'test_autonomy_observation_runtime.AutonomyObservationRuntimeTests.test_sparse_valid_surface_recovers_without_a_passive_wait_becoming_actuator_failure'],
    'native':['test_native_sensing'],
    'connections':['test_navigation_disconnect','test_resident_client','test_portable_pr_review','test_connection_roaming','test_shared_listener.SharedListenerTests','test_robot_discovery','test_connection_pairing','test_connection_relay'],
    'ui':[],
}
POSIX_GROUPS={'maps','lifecycle','native'}
EXCLUDE_TESTS={
    'test_portable_pr_review.PortableReviewTests.test_windows_proxy_executes_absolute_helper_and_preserves_arguments':
        'Executes installed OpenSSH and a synthetic helper; covered separately by the Windows MSVC/SSH job.',
    'test_portable_pr_review.PortableReviewTests.test_windows_openssh_path_option_round_trip_without_network':
        'Executes installed OpenSSH; covered separately by the real Windows MSVC/SSH job.',
    'test_pose_monitor.PoseMonitorTests.test_actual_unready_pose_series_is_never_frozen_into_a_trusted_marker':
        'Private recorded pose replay; deliberately excluded even when locally available.',
    'test_mapping_request_identity.MappingRequestIdentityTests.test_lost_start_reply_retry_does_not_spawn_again':
        'Calls the host process inventory without a test-local proc fixture; outside portable effect scope.',
    'test_mapping_request_identity.MappingRequestIdentityTests.test_stop_waits_for_start_then_saves_same_capture':
        'Calls the host process inventory without a test-local proc fixture; outside portable effect scope.',
}
JS_TESTS=['test_disconnect_policy_ui.cjs','test_voice_control_ui.cjs','test_autonomy_ui.cjs','test_autonomy_map_view.cjs','test_navigation_preparation.cjs','test_navigation_diagnostics.cjs','test_goal_approach_ui.cjs','test_navigation_ui.cjs','test_cloud_ui.cjs','test_camera_player.cjs',
          'test_model_check_ui.cjs','test_navigation_status.cjs','test_pose_display.cjs','test_connection_roaming_ui.cjs','test_map_library_ui.cjs','test_map_library_review.cjs','test_map_workflows_review.cjs','test_connection_onboarding_ui.cjs','test_product_ui_copy.cjs']
NATIVE_PROOFS=[
    'test_native_sensing.NativeSensingTests.test_random_frames_require_native_hit_and_exact_all_field_equality',
    'test_native_sensing.NativeSensingTests.test_full_100k_150k_300k_inputs_are_native_not_hidden_fallback',
]
NODE_GUARD=r'''"use strict";
const blocked=[];
const deny=name=>function(){blocked.push(name);throw Error("Portable checks forbid "+name);};
const net=require("node:net");net.Socket.prototype.connect=deny("net.connect");net.Server.prototype.listen=deny("net.listen");
const dgram=require("node:dgram");dgram.Socket.prototype.send=deny("dgram.send");dgram.Socket.prototype.bind=deny("dgram.bind");
const dns=require("node:dns");for(const key of Object.keys(dns))if(/^(lookup|resolve|reverse)/.test(key)&&typeof dns[key]==="function")dns[key]=deny("dns."+key);
const dp=require("node:dns/promises");for(const key of Object.keys(dp))if(typeof dp[key]==="function")dp[key]=deny("dns.promises."+key);
for(const name of ["node:http","node:https"]){const m=require(name);m.request=deny(name+".request");m.get=deny(name+".get");}
const child=require("node:child_process");for(const name of ["exec","execFile","execSync","execFileSync","spawn","spawnSync","fork"])child[name]=deny("child_process."+name);
globalThis.fetch=deny("fetch");if(globalThis.WebSocket)globalThis.WebSocket=deny("WebSocket");
const expected=["net.connect","child_process.spawn"];
for(const fn of [()=>net.Socket.prototype.connect.call({}),()=>child.spawn("not-executed")]){try{fn();}catch(error){}}
if(JSON.stringify(blocked)!==JSON.stringify(expected))throw Error("Portable Node effect guard self-test failed");
blocked.length=0;
process.once("exit",()=>process.stderr.write("PORTABLE_NODE_GUARD="+JSON.stringify({blocked,self_test:expected})+"\n"));
'''


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def declared_tests(path):
    """Static IDs, including locally inherited methods; never imports tests."""
    tree=ast.parse(path.read_text(encoding='utf-8-sig'),filename=path.name)
    classes={node.name:node for node in tree.body if isinstance(node,ast.ClassDef)}
    def methods(name,seen=()):
        if name not in classes or name in seen:return set()
        node=classes[name];result={child.name for child in node.body if isinstance(child,(ast.FunctionDef,ast.AsyncFunctionDef)) and child.name.startswith('test_')}
        for base in node.bases:
            if isinstance(base,ast.Name):result|=methods(base.id,seen+(name,))
        return result
    return sorted(path.stem+'.'+name+'.'+method for name in classes for method in methods(name))


def inventory():
    return {path.name:declared_tests(path) for path in sorted((ROOT/'tests').glob('test_*.py'))}


def selected_ids(group,available):
    result=[]
    for selector in GROUPS[group]:
        module=selector.split('.')[0]
        if module+'.py' not in available:raise ValueError('Missing allowlisted test source: '+module+'.py')
        ids=[name for name in available[module+'.py'] if name==selector or name.startswith(selector+'.')]
        if not ids:raise ValueError('Allowlisted selector contains no declared tests: '+selector)
        result.extend(name for name in ids if name not in EXCLUDE_TESTS)
    return sorted(set(result))


def source_files():
    files=set(ROOT.glob('*.py'))|set(ROOT.glob('*.js'))
    for name in ['m20','bridge','tools','tests','windows']:
        files.update((ROOT/name).rglob('*.py'))
    for pattern in ['*.js','*.css','*.html']:files.update((ROOT/'platform_web').glob(pattern))
    files.update((ROOT/'tests').glob('test_*.cjs'));files.update((ROOT/'native').glob('*.cpp'))
    for name in ['package.json','package-lock.json','requirements-dev.txt','reference/errors.md','platform_web/assets/model/M20.urdf']:
        path=ROOT/name
        if path.is_file():files.add(path)
    return sorted(path for path in files if path.is_file())


def clean_environment(temporary):
    env={key:value for key,value in os.environ.items() if not key.startswith(('M20_','TYPESAFE_','SSH_','AWS_','DYLD_'))
         and key not in ['PYTHONPATH','PYTHONHOME','PYTHONSTARTUP','PYTHONINSPECT','LD_PRELOAD']
         and not key.endswith(('_TOKEN','_PASSWORD','_SECRET','_API_KEY'))}
    env.update(YOLO_OFFLINE='true',YOLO_AUTOINSTALL='false',YOLO_CONFIG_DIR=str(temporary/'model-settings'),
               OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',VECLIB_MAXIMUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
    return env


def prepare_host_metadata():
    """Warm only stdlib OS metadata before the test effect guard is installed.

    CPython on Windows may execute the read-only ``ver`` builtin while filling
    platform.uname's cache. Limit this preparation to those exact stdlib probes;
    no product, NumPy or other optional dependency is imported here.
    """
    queries=[];original=subprocess.Popen
    def metadata_popen(*args,**kwargs):
        command=args[0] if args else kwargs.get('args')
        if sys.platform!='win32' or kwargs.get('shell') is not True or command not in ('ver','command /c ver','cmd /c ver'):
            raise RuntimeError('Unexpected subprocess during host-metadata preparation')
        queries.append('windows_version_builtin_ver')
        return original(*args,**kwargs)
    subprocess.Popen=metadata_popen
    try:
        row=platform.uname();machine=platform.machine()
    finally:subprocess.Popen=original
    return dict(stage='before_test_effect_guard',provider='stdlib.platform',queries=['uname','machine'],
                values=dict(system=row.system,release=row.release,version=row.version,machine=machine),
                preparation_subprocesses=queries,hostname_recorded=False)


def verify_host_metadata_cache(prepared):
    """Called with Popen/network denial already active, before product imports."""
    row=platform.uname()
    values=dict(system=row.system,release=row.release,version=row.version,machine=platform.machine())
    if values!=prepared['values']:raise RuntimeError('Host metadata cache changed after guard installation')
    return dict(passed=True,stage='after_test_effect_guard',queries=['uname','machine'])


def python_guard():
    blocked=[];suppressed=[]
    forbidden={'socket.connect','socket.bind','socket.sendto','socket.getaddrinfo','socket.gethostbyname',
               'socket.gethostbyaddr','subprocess.Popen','os.system','os.posix_spawn','os.exec','pty.spawn'}
    private=[str((ROOT/'data').resolve()),str((ROOT/'tests/fixtures/runtime_replay').resolve())]
    def audit(event,args):
        if event=='subprocess.Popen' and args and args[0]=='lscpu' and args[1] in (['lscpu'],'lscpu'):
            suppressed.append('numpy_optional_lscpu');raise FileNotFoundError('Optional CPU probe disabled')
        if event=='open' and args and isinstance(args[0],(str,bytes,os.PathLike)):
            supplied=os.fsdecode(args[0])
            # open's audit payload omits dir_fd. A bare name can belong to a
            # temporary fixture directory being removed, not to the cwd.
            if not os.path.isabs(supplied) and len(Path(supplied).parts)==1:return
            path=str(Path(supplied).resolve())
            if any(path==base or path.startswith(base+os.sep) for base in private):
                blocked.append('private_runtime_or_recording_access');raise RuntimeError('Portable checks cannot read runtime data or recorded fixtures')
            if re.fullmatch(r'/proc/[^/]+/(cmdline|stat)',path):
                blocked.append('host_process_inventory_access');raise RuntimeError('Portable checks require a test-local process fixture')
        if event in forbidden:
            blocked.append(event);raise RuntimeError('Portable checks forbid '+event)
    sys.addaudithook(audit)
    for event in ['socket.connect','socket.sendto','subprocess.Popen']:
        try:sys.audit(event,None)
        except RuntimeError:pass
        else:raise AssertionError('Effect guard self-test failed')
    try:sys.audit('open',str(ROOT/'data'/'guard-probe'),'r',0)
    except RuntimeError:pass
    else:raise AssertionError('Private-data guard self-test failed')
    expected=list(blocked);blocked.clear()
    return blocked,suppressed,expected


def worker(group,native_library):
    try:host_metadata=prepare_host_metadata()
    except Exception as exc:
        print(MARKER+json.dumps(dict(status='failed',tests_run=0,passed=[],failed=[],errors=['host_metadata_prepare: '+str(exc)],skipped=[]),ensure_ascii=False));return 1
    blocked,suppressed,self_test=python_guard();sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
    os.environ['YOLO_OFFLINE']='true';os.environ['YOLO_AUTOINSTALL']='false'
    if native_library:os.environ['M20_SENSING_LIBRARY']=native_library
    if group=='native':os.environ['M20_REQUIRE_NATIVE_SENSING']='1'
    import unittest
    class Result(unittest.TextTestResult):
        def __init__(self,*args,**kwargs):super().__init__(*args,**kwargs);self.passed=[]
        def addSuccess(self,test):super().addSuccess(test);self.passed.append(test.id())
    try:
        host_metadata['post_guard_cache_check']=verify_host_metadata_cache(host_metadata)
        import cv2
        def no_capture(*args,**kwargs):blocked.append('cv2.VideoCapture');raise RuntimeError('Portable checks forbid camera capture')
        cv2.VideoCapture=no_capture
        ids=selected_ids(group,inventory());suite=unittest.TestLoader().loadTestsFromNames(ids)
        native_before=None
        if group=='native':
            import m20.native_sensing as native_sensing
            native_before=native_sensing.backend_status()
            if not native_before['available']:raise RuntimeError('Explicit native build did not load')
        result=unittest.TextTestRunner(verbosity=1,resultclass=Result).run(suite)
        state='failed' if not result.wasSuccessful() or blocked else 'not_verified' if result.skipped else 'passed'
        payload=dict(status=state,selected_test_ids=ids,tests_run=result.testsRun,passed=result.passed,
                     failed=[test.id() for test,_ in result.failures],errors=[test.id() for test,_ in result.errors],
                     skipped=[dict(test=test.id(),reason=reason) for test,reason in result.skipped],
                     blocked_effects=blocked,suppressed_probes=suppressed,guard_self_test=self_test)
        if group=='native':
            after=native_sensing.backend_status()
            proof=dict(available=after['available'],abi=after['abi'],native_frames=after['native_frames']-native_before['native_frames'],
                       fallback_frames=after['fallback_frames']-native_before['fallback_frames'],fallback_reasons=after['fallback_reasons'],
                       required_native_test_ids=NATIVE_PROOFS,all_required_passed=all(name in result.passed for name in NATIVE_PROOFS),
                       library_sha256=digest(Path(after['library'])))
            payload['native_proof']=proof
            if not proof['available'] or proof['native_frames']<=0 or not proof['all_required_passed']:payload['status']='failed'
    except Exception as exc:
        payload=dict(status='failed',tests_run=0,passed=[],failed=[],errors=[type(exc).__name__+': '+str(exc)],skipped=[],
                     blocked_effects=blocked,suppressed_probes=suppressed,guard_self_test=self_test)
    payload['host_metadata']=host_metadata
    print(MARKER+json.dumps(payload,ensure_ascii=False))
    return 0 if payload['status']=='passed' else 2 if payload['status']=='not_verified' else 1


def run(command,cwd,environment,timeout):
    began=time.monotonic()
    try:
        result=subprocess.run(command,cwd=cwd,env=environment,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=timeout)
        return dict(returncode=result.returncode,elapsed_s=round(time.monotonic()-began,3),stdout=result.stdout,stderr=result.stderr)
    except (OSError,subprocess.TimeoutExpired) as exc:
        return dict(returncode=None,elapsed_s=round(time.monotonic()-began,3),stdout='',stderr=type(exc).__name__+': '+str(exc))


def run_python(group,stage,env,library,timeout):
    command=[sys.executable,'-I','-B','-X','utf8',str(stage/'tools/check_portable.py'),'--_worker',group]
    if library:command+=['--_native-library',str(library)]
    execution=run(command,stage,env,timeout)
    for line in reversed(execution['stdout'].splitlines()):
        if line.startswith(MARKER):
            result=json.loads(line[len(MARKER):]);result.update(elapsed_s=execution['elapsed_s'],returncode=execution['returncode']);return result,execution
    return dict(status='failed',tests_run=0,error='Worker did not return a result',returncode=execution['returncode']),execution


def run_ui(stage,env,node,guard,timeout):
    command=[node,'--require',str(guard),'-e','for(const file of process.argv.slice(1))require(file)']+[str(stage/'tests'/name) for name in JS_TESTS]
    execution=run(command,stage,env,timeout);guard_result=None
    for line in execution['stderr'].splitlines():
        if line.startswith('PORTABLE_NODE_GUARD='):guard_result=json.loads(line.split('=',1)[1])
    def count(key):
        matches=re.findall(r'^# '+key+r' (\d+)\s*$',execution['stdout'],re.M);return int(matches[-1]) if matches else None
    totals={key:count(key) for key in ['tests','pass','fail','skipped','cancelled']}
    passed=execution['returncode']==0 and guard_result is not None and guard_result['blocked']==[] and totals['tests'] and totals['fail']==0
    state='failed' if not passed else 'not_verified' if totals['skipped'] or totals['cancelled'] else 'passed'
    result=dict(status=state,totals=totals,files=JS_TESTS,guard=guard_result,elapsed_s=execution['elapsed_s'],returncode=execution['returncode'])
    return result,execution


def scrub(value,temporary=None):
    if isinstance(value,dict):return {key:scrub(item,temporary) for key,item in value.items()}
    if isinstance(value,list):return [scrub(item,temporary) for item in value]
    if isinstance(value,str):
        replacements=[(str(ROOT),'<repo>'),(str(Path.home()),'<home>')]
        if temporary:replacements.extend([(str(temporary.resolve()),'<temporary>'),(str(temporary),'<temporary>')])
        for old,new in sorted(replacements,key=lambda pair:len(pair[0]),reverse=True):
            value=value.replace(old,new).replace(old.replace('\\','/'),new)
        return value
    return value


def main(args):
    groups=list(dict.fromkeys(args.groups.split(','))) if args.groups else list(GROUPS)
    if any(group not in GROUPS for group in groups):raise ValueError('Unknown group; choose '+','.join(GROUPS))
    if not 1<=args.jobs<=8:raise ValueError('--jobs must be 1..8')
    available=inventory();selected={group:selected_ids(group,available) for group in groups if group!='ui'}
    chosen={name for ids in selected.values() for name in ids};all_ids={name for ids in available.values() for name in ids}
    omitted=[dict(test=name,reason=EXCLUDE_TESTS.get(name,'Private recorded map replay' if name.startswith('test_mapping_save_outcomes.SavedObservationTests.') else 'Outside the reviewed portable selection')) for name in sorted(all_ids-chosen)]
    files=source_files();before={p.relative_to(ROOT).as_posix():digest(p) for p in files}
    host_metadata=prepare_host_metadata()
    report=dict(schema_version=1,scope='allowlisted_portable_offline_checks_not_field_acceptance',
                created_at=datetime.now(timezone.utc).isoformat(),platform=sys.platform,machine=host_metadata['values']['machine'],python=platform.python_version(),
                host_metadata=host_metadata,
                selected_groups=groups,language_effect_guards=True,os_network_sandbox=False,
                private_fixtures_copied=0,model_weights_copied=0,source_hashes=before,
                selection=dict(selected_python_methods=len(chosen),omitted_python_methods=len(omitted),omitted=omitted,
                    omitted_javascript_files=[p.name for p in (ROOT/'tests').glob('test_*.cjs') if 'ui' not in groups or p.name not in JS_TESTS],
                    excluded_private_replay_scope='tests/fixtures/runtime_replay/*.json; strict recorded acceptance is separate'),
                results={},versions={})
    for name in ['numpy','opencv-python','opencv-python-headless']:
        try:report['versions'][name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:pass
    syntax=[]
    for path in files:
        if path.suffix=='.py':
            try:ast.parse(path.read_text(encoding='utf-8-sig'),filename=path.relative_to(ROOT).as_posix())
            except (SyntaxError,UnicodeError) as exc:syntax.append(dict(file=path.relative_to(ROOT).as_posix(),error=type(exc).__name__))
    report['python_syntax_errors']=syntax
    output=Path(args.report).absolute();output.parent.mkdir(parents=True,exist_ok=True)
    logs=output.parent/'logs';logs.mkdir(exist_ok=True);unsupported=[]
    with tempfile.TemporaryDirectory(prefix='m20-portable-') as directory:
        temporary=Path(directory);stage=temporary/'source';stage.mkdir()
        for path in files:
            dest=stage/path.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)
        env=clean_environment(temporary);node=shutil.which('node');library=None
        guard=temporary/'node_guard.cjs';guard.write_text(NODE_GUARD,encoding='utf-8')
        if node:
            node_version=run([node,'--version'],stage,env,15);report['versions']['node']=node_version['stdout'].strip()
        if 'native' in groups:
            if sys.platform not in ['darwin','linux']:
                report['native_build']=dict(status='unsupported',reason='Windows native parity runs in its dedicated MSVC CI job; this guarded native group supports POSIX only')
                unsupported.append('native')
            else:
                library=temporary/'native/libsensing.so'
                execution=run([sys.executable,'-I','-B','-X','utf8',str(stage/'tools/build_native_sensing.py'),'--output',str(library)],stage,env,120)
                try:build=json.loads(execution['stdout'])
                except ValueError:build=dict(ok=False,error=execution['stderr'])
                build['status']='passed' if execution['returncode']==0 and build.get('ok') and library.is_file() and build.get('source_sha256')==before.get('native/sensing.cpp') else 'failed'
                report['native_build']=scrub(build,temporary)
                (logs/'native-build.txt').write_text(scrub(execution['stdout']+'\n'+execution['stderr'],temporary),encoding='utf-8')
                if build['status']!='passed':library=None
        else:report['native_build']=dict(status='omitted',reason='Native group was not selected; no native result is claimed')
        if 'ui' in groups:
            js_errors=[]
            if node:
                for path in sorted((stage/'platform_web').glob('*.js')):
                    execution=run([node,'--check',str(path)],stage,env,20)
                    if execution['returncode']!=0:js_errors.append(dict(file=path.relative_to(stage).as_posix(),error=execution['stderr']))
            else:js_errors.append(dict(error='Node.js is not installed'))
            report['javascript_syntax_errors']=scrub(js_errors,temporary)
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures={}
            for group in groups:
                if os.name=='nt' and group in POSIX_GROUPS:
                    unsupported.append(group);report['results'][group]=dict(status='unsupported',tests_run=0,reason='POSIX/native group not verified on Windows');continue
                if group=='native' and library is None:
                    report['results'][group]=dict(status='failed',tests_run=0,reason='Required explicit native build failed');continue
                if group=='ui':
                    if not node:report['results'][group]=dict(status='unavailable',tests_run=0,reason='Node.js missing');continue
                    missing=[name for name in JS_TESTS if not (stage/'tests'/name).is_file()]
                    if missing:report['results'][group]=dict(status='failed',missing_files=missing);continue
                    futures[pool.submit(run_ui,stage,env,node,guard,args.timeout)]=group
                else:futures[pool.submit(run_python,group,stage,env,library,args.timeout)]=group
            for future in as_completed(futures):
                group=futures[future]
                try:result,execution=future.result()
                except Exception as exc:result=dict(status='failed',error=type(exc).__name__+': '+str(exc));execution=dict(stdout='',stderr=str(exc))
                report['results'][group]=scrub(result,temporary)
                (logs/(group+'.txt')).write_text(scrub(execution['stdout']+'\n'+execution['stderr'],temporary),encoding='utf-8')
                print(group+': '+result['status'],flush=True)
        report['staged_source_changed']=[name for name,value in before.items() if not (stage/name).is_file() or digest(stage/name)!=value]
    report['working_source_changed']=[name for name,value in before.items() if not (ROOT/name).is_file() or digest(ROOT/name)!=value]
    report['unsupported_groups']=sorted(set(unsupported))
    results=report['results'];statuses=[result.get('status') for result in results.values()]
    for group,ids in selected.items():
        result=results[group];result.setdefault('selected_test_ids',ids)
        attempted=set(result.get('passed',[]))|set(result.get('failed',[]))|set(result.get('errors',[]))|{row['test'] for row in result.get('skipped',[])}
        result['unexecuted_test_ids']=sorted(set(ids)-attempted)
    failures=bool(syntax or report.get('javascript_syntax_errors') or 'failed' in statuses)
    incomplete=bool(report['working_source_changed'] or report['staged_source_changed'] or any(state not in ['passed','failed'] for state in statuses))
    report['status']='failed' if failures else 'not_verified' if incomplete else 'passed' if set(groups)==set(GROUPS) else 'partial'
    report['summary']=dict(python_tests_run=sum(r.get('tests_run',0) for k,r in results.items() if k!='ui'),
        python_skipped=sum(len(r.get('skipped',[])) for k,r in results.items() if k!='ui'),
        unexecuted_selected_python_tests=sum(len(r.get('unexecuted_test_ids',[])) for k,r in results.items() if k!='ui'),
        javascript_tests_run=results.get('ui',{}).get('totals',{}).get('tests',0),
        javascript_skipped=results.get('ui',{}).get('totals',{}).get('skipped',0),
        omitted_python_methods=len(omitted),omitted_javascript_files=len(report['selection']['omitted_javascript_files']),
        omitted_groups=sorted(set(GROUPS)-set(groups)),unsupported_groups=len(set(unsupported)))
    output.write_text(json.dumps(scrub(report),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(status=report['status'],summary=report['summary'],report=scrub(str(output))),ensure_ascii=False))
    return 1 if failures else 2 if incomplete else 0


def parse_args():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups',help='Comma-separated reviewed groups: '+','.join(GROUPS))
    parser.add_argument('--report',default='.artifacts/portable/report.json')
    parser.add_argument('--jobs',type=int,default=2);parser.add_argument('--timeout',type=int,default=300)
    parser.add_argument('--_worker',choices=[x for x in GROUPS if x!='ui'],help=argparse.SUPPRESS)
    parser.add_argument('--_native-library',help=argparse.SUPPRESS)
    return parser.parse_args()


if __name__=='__main__':
    args=parse_args()
    if args._worker:sys.exit(worker(args._worker,args._native_library))
    try:sys.exit(main(args))
    except Exception as exc:
        failure=dict(status='failed',scope='portable_runner_setup',error=scrub(type(exc).__name__+': '+str(exc)))
        output=Path(args.report);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(failure,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(failure,ensure_ascii=False));sys.exit(1)
