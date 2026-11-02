#!/usr/bin/env python3
"""Strictly local M20 acceptance: default inventory + recorded-evidence replay.

No live mode, API URL, SSH host or deployment option exists. Test workers run
under macOS deny-network sandboxing, plus Python/Node effect guards. The default
quick check needs only the standard library. --tests uses the runtime venv.

Examples:
  python3 tools/offline_acceptance.py
  data/runtime-venv/bin/python tools/offline_acceptance.py --tests
  data/runtime-venv/bin/python tools/offline_acceptance.py --tests --record-baseline

Exit 0: selected offline checks passed (quick requires a matching tested baseline).
Exit 1: a check/test failed or an effect was attempted. Exit 2: not fully verified.
An offline pass is never physical robot, deployment or whole-floor acceptance.
"""
import argparse
import ast
from email.parser import Parser
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
FIXTURES=ROOT/'tests/fixtures/runtime_replay'
OUTPUT=ROOT/'data/reviews/autonomy-v3-20261101/acceptance'
BASELINE=OUTPUT/'tested-source-baseline.json'
MARKER='OFFLINE_WORKER_RESULT='
SANDBOX=Path('/usr/bin/sandbox-exec')
SANDBOX_PROFILE='(version 1) (allow default) (deny network*)'
GROUPS={
    'motion':['test_autonomy','test_autonomy_recovery','test_exploration_frontiers','test_exploration_control_v3','test_local_trajectory','test_sensor_cache','test_sensing_equivalence','test_native_sensing','test_terrain_profiles','test_control_clock',
              'test_platform.PlatformTests.test_bounded_relative_control_expires_without_next_http_request'],
    'navigation':['test_point_goal','test_navigation_planner','test_navigation_planning_v3','test_point_navigation_runtime','test_live_traversability','test_live_surface_localization','test_navigation_lifecycle','test_navigation_diagnostics'],
    'lifecycle':['test_autonomy','test_localization_runtime','test_mapping_request_identity','test_operations_lifecycle','test_delivery_boundary','test_platform_instance'],
    'localization':['test_localization_recovery','test_localization_runtime','test_mapping_observation'],
    'maps':['test_map_change_policy','test_map_fusion','test_map_learning','test_map_update_v3','test_mapping_save_outcomes'],
    'ui':[],
}
JS_TESTS=['test_autonomy_ui.cjs','test_navigation_ui.cjs','test_cloud_ui.cjs',
          'test_camera_player.cjs','test_model_check_ui.cjs','test_navigation_status.cjs']
REPLAY_FIXTURES=['native_zero_progress.json','narrow_forward_scan.json','conflicting_map_candidate.json','localization_contract.json']
REQUIRED={
    'frontier_information_gain_and_coverage':['test_two_rooms_with_occluded_doorway_reach_98_percent_coverage',
        'test_three_rooms_with_offset_doors_complete_without_dense_rescanning','test_two_occluded_rooms_with_continuous_commands'],
    'continuous_point_navigation':['test_narrow_doorway_with_full_body','test_static_wall_requires_a_detour',
        'test_new_obstacle_is_avoided_without_changing_static_walls','test_reverse_in_narrow_corridor',
        'test_real_start_prepare_drive_arrival_waits_for_actual_stop','test_old_native_cancel_ack_cannot_confirm_new_platform_stop',
        'test_late_old_preparation_cannot_replace_new_task_planner','test_repeated_heading_oscillation_cannot_keep_zero_translation_alive'],
    'structural_map_updates':['test_minor_consistent_bias_is_corrected_instead_of_thickening_walls',
        'test_complete_passive_update_preserves_old_map_and_raw_observations','test_same_structural_correction_applies_to_original_map_extension',
        'test_body_pose_is_never_silently_used_as_sensor_origin','test_occlusion_and_unseen_regions_are_preserved'],
    'bounded_escape_and_return_to_progress':['test_closed_loop_sidesteps_then_resumes_progress',
        'test_recovery_stops_if_person_enters_selected_side',
        'test_unknown_side_is_not_a_recovery_corridor',
        'test_no_feedback_progress_expires_attempt_and_is_bounded',
        'test_reached_intermediate_vantage_can_continue_in_same_decision',
        'test_stationary_pose_noise_is_not_reported_as_walked_distance',
        'test_forward_momentum_is_braked_before_side_escape',
        'test_missing_motion_feedback_does_not_start_a_new_escape'],
    'background_task_start_failure':['test_thread_start_failure_does_not_leave_running_task_or_consume_slots'],
    'source_delivery_boundary':['test_real_replays_and_runtime_profiles_are_not_source_delivery'],
    'monotonic_control_and_feedback':['test_wall_clock_rollback_cannot_extend_moving_lease',
        'test_fresh_lease_is_not_lost_on_forward_wall_clock_jump',
        'test_expiry_repeats_zero_for_a_bounded_burst',
        'test_fresh_commands_cannot_override_stale_robot_feedback',
        'test_unknown_or_future_legacy_timestamp_is_not_fresh'],
    'no_fake_motion_from_jitter':['test_changing_goals_and_noisy_coverage_cannot_keep_turning_alive',
        'test_static_feedback_does_not_turn_slam_drift_into_movement'],
    'mapping_request_ownership':['test_lost_start_reply_retry_does_not_spawn_again',
        'test_cleanup_cannot_stop_another_computers_capture',
        'test_stop_waits_for_start_then_saves_same_capture',
        'test_manual_start_also_returns_identity_for_later_continue',
        'test_legacy_mapping_service_is_rejected_before_start_dispatch'],
    'native_zero_progress_handling':['test_real_zero_progress_trace_waits_then_cancels_once',
        'test_real_heading_progress_counts_but_oscillation_does_not'],
    'native_read_only_diagnostics':['test_normal_status_with_repeated_zero_output_is_not_claimed_as_moving',
        'test_current_idle_does_not_reuse_old_active_failure',
        'test_unknown_log_format_is_unavailable_not_healthy',
        'test_reader_only_reads_bounded_log_and_does_not_modify_it'],
    'native_prepare_feedback':['test_goal_waits_for_mode_and_gait_feedback_in_order',
        'test_cancel_during_preparation_cannot_restart_on_late_feedback',
        'test_mode_or_gait_ack_without_stationary_feedback_cannot_start',
        'test_slow_map_validation_is_cancellable_before_any_mode_or_goal',
        'test_slow_resume_cannot_restart_after_operator_cancel'],
    'stale_navigation_receipts':['test_restarted_platform_does_not_reuse_value_or_accept_old_success',
        'test_counter_is_durable_before_goal_emission'],
    'late_scan_zero_and_recovery':[
        'test_scan_expiring_during_planning_cannot_refresh_motion',
        'test_late_next_packet_retains_fresh_scan_without_renewing_it',
        'test_expired_cache_cannot_survive_late_next_packet',
        'test_bounded_relative_control_expires_without_next_http_request'],
    'narrow_straight_passage':['test_start_can_use_straight_passage_without_in_place_turn_space'],
    'map_conflict_and_preservation':['test_floor_overlap_cannot_publish_a_displaced_wall',
        'test_fusion_keeps_original_and_raw_history_and_adds_a_version',
        'test_fusion_keeps_each_unobserved_old_point_without_voxel_averaging'],
    'independent_localization':['test_new_scan_verifies_but_same_or_older_stamp_cannot',
        'test_symmetric_room_does_not_choose_arbitrary_heading',
        'test_motion_or_failed_independent_frame_cannot_emit_initialization',
        'test_saved_trajectory_runs_before_local_search_and_only_initializes_pose'],
    'temporal_environment_updates':['test_same_extension_confirms_new_obstacle_from_saved_keyframes',
        'test_same_extension_clears_old_obstacle_without_adding_early_ghost_back',
        'test_object_seen_for_three_epochs_then_removed_does_not_leave_ghost_cloud'],
    'cancelled_preparation':['test_cancel_before_mapping_dispatch_never_starts_collection',
        'test_cancel_during_mapping_dispatch_stops_and_saves_once',
        'test_prepare_expiry_cannot_start_slam_after_watchdog',
        'test_preparation_waits_for_collection_not_only_launch_ack',
        'test_wrong_mapping_request_is_never_stopped_as_cleanup'],
    'cancelled_relocation':['test_relocation_cancel_does_not_initialize_or_start_vendor_fallback',
        'test_late_success_after_cancel_is_not_reported_as_current',
        'test_connection_switch_before_vendor_fallback_never_targets_new_robot'],
    'reconnect_and_result_ownership':['test_connection_switch_cannot_retarget_old_preparation',
        'test_old_save_completion_cannot_overwrite_new_session'],
    'continue_existing_capture':['test_continue_current_capture_checks_identity_without_starting_it_again',
        'test_continue_legacy_capture_without_identity_does_not_move_or_stop_it',
        'test_cancel_continue_preparation_keeps_existing_capture',
        'test_explicit_save_during_continue_preparation_uses_existing_owner'],
    'unclaimed_or_unknown_work':['test_external_navigation_is_not_claimed_by_autonomy_after_restart',
        'test_worker_start_failure_does_not_leave_preparing_task',
        'test_unknown_preparation_readback_unlocks_start_without_reissuing'],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_inventory():
    paths=set(ROOT.glob('*.py'))|set((ROOT/'bridge').glob('*.py'))
    paths|=set((ROOT/'platform_web').glob('*.js'))|set((ROOT/'platform_web').glob('*.css'))
    paths|=set((ROOT/'tests').glob('test_*.py'))|set((ROOT/'tests').glob('test_*.cjs'))
    paths|=set((ROOT/'tools').glob('*.py'))
    paths|=set((ROOT/'native').glob('*.cpp'))|set((ROOT/'native').glob('*.so'))
    paths|={ROOT/'platform_web/index.html',Path(__file__).resolve()}
    paths|=set(ROOT.glob('requirements*.txt'))|{ROOT/'package.json',ROOT/'package-lock.json'}
    paths|=set(FIXTURES.glob('*.json'))|set(FIXTURES.glob('*.cjs'))
    paths|=set((ROOT/'tests/fixtures').glob('*.json'))
    return {p.relative_to(ROOT).as_posix():digest(p) for p in sorted(paths) if p.is_file()}


def runtime_inventory():
    """Read installed package metadata without importing native/model modules."""
    venv=ROOT/'data/runtime-venv';config=venv/'pyvenv.cfg'
    wanted={'numpy','opencv-python','opencv-python-headless','ultralytics','torch','lap'}
    packages={};sites=sorted((venv/'lib').glob('python*/site-packages')) if venv.is_dir() else []
    for site in sites:
        for meta in sorted(site.glob('*.dist-info/METADATA')):
            with meta.open(errors='replace') as source:header=Parser().parsestr(source.read(65536),headersonly=True)
            name=(header.get('Name') or '').lower().replace('_','-')
            if name in wanted:packages[name]=dict(version=header.get('Version'),metadata_sha256=digest(meta))
    return dict(platform_venv_present=venv.is_dir(),pyvenv_config_sha256=digest(config) if config.is_file() else None,
                packages=packages,note='Package metadata only; this does not hash every dependency binary.')


def changed(before,after):
    return sorted(p for p in before.keys()|after.keys() if before.get(p)!=after.get(p))


def source_observation(inventory):
    result={}
    for name in inventory:
        try:
            stat=(ROOT/name).stat();result[name]=[stat.st_size,stat.st_mtime_ns,stat.st_ino]
        except OSError:result[name]=None
    return result


def syntax_check(inventory):
    failures=[]
    for name in inventory:
        if not name.endswith('.py'):continue
        try:ast.parse((ROOT/name).read_text(),filename=name)
        except (OSError,SyntaxError,UnicodeError) as exc:
            failures.append(dict(file=name,error=type(exc).__name__,line=getattr(exc,'lineno',None)))
    return failures


def evidence_manifest():
    result=[]
    for file in sorted(FIXTURES.glob('*.json')):
        record=json.loads(file.read_text());origins=[]
        for item in record.get('provenance',[]):
            path=ROOT/item['path']
            origins.append(dict(path=item['path'],available=path.is_file(),
                                source_hash_matches=digest(path)==item['sha256'] if path.is_file() else None))
        result.append(dict(fixture=file.relative_to(ROOT).as_posix(),kind=record['kind'],sources=origins))
    return result


def clean_environment():
    # No robot endpoint or credential is needed by a fixture-based test.
    environment={k:v for k,v in os.environ.items()
                 if not k.startswith(('M20_','TYPESAFE_','SSH_','AWS_'))
                 and not k.endswith(('_TOKEN','_PASSWORD','_SECRET','_API_KEY'))}
    environment['PYTHONPATH']=os.pathsep.join([str(ROOT),str(ROOT/'tests')])
    environment['PYTHONDONTWRITEBYTECODE']='1'
    environment['YOLO_OFFLINE']='true'
    environment['YOLO_AUTOINSTALL']='false'
    return environment


def sandbox_command(command):
    if sys.platform!='darwin' or not SANDBOX.is_file():
        raise RuntimeError('OS network-denial sandbox unavailable; test execution refused. Inventory remains available.')
    return [str(SANDBOX),'-p',SANDBOX_PROFILE]+command


def run_command(command,timeout=180):
    started=time.monotonic()
    try:
        with tempfile.TemporaryDirectory(prefix='.worker-',dir=OUTPUT) as temporary:
            environment=clean_environment()
            # Do not read or change the user's global Ultralytics settings/API
            # credentials while importing the optional model package.
            environment['YOLO_CONFIG_DIR']=str(Path(temporary)/'model-settings')
            process=subprocess.run(sandbox_command(command),cwd=ROOT,env=environment,
                                   capture_output=True,text=True,timeout=timeout)
        return dict(returncode=process.returncode,elapsed_s=round(time.monotonic()-started,3),
                    stdout=process.stdout,stderr=process.stderr,os_network_denied=True)
    except (RuntimeError,subprocess.TimeoutExpired,OSError) as exc:
        return dict(returncode=None,elapsed_s=round(time.monotonic()-started,3),
                    stdout='',stderr=str(exc),os_network_denied=False)


def python_guard():
    blocked=[];suppressed=[]
    forbidden={'socket.connect','socket.bind','socket.sendto','socket.getaddrinfo',
               'socket.gethostbyname','socket.gethostbyaddr','subprocess.Popen',
               'os.system','os.posix_spawn','os.exec','pty.spawn'}
    def audit(event,args):
        if event=='subprocess.Popen' and args[0]=='lscpu' and args[1] in (['lscpu'],'lscpu'):
            # NumPy 1.26 testing optionally probes Linux SVE support on import.
            # Suppress that probe exactly as an unavailable utility, without
            # allowing any child process or masking another requested command.
            suppressed.append('numpy_optional_lscpu');raise FileNotFoundError('Offline: optional CPU probe unavailable')
        if event in forbidden:
            blocked.append(event)
            raise RuntimeError('Offline acceptance forbids '+event)
    sys.addaudithook(audit)
    return blocked,suppressed


def recorded_replay():
    """Assertions against recorded facts and pure production functions only."""
    from autonomy_core import clearance,Explorer
    from map_change_policy import gate_alignment
    from localization_runtime import AutomaticLocalizer
    from types import SimpleNamespace
    from bridge.ros_sensors import RosSensors
    import threading
    results=[]
    def check(name,fn,kind='pure_code_replay'):
        try:
            details=fn();results.append(dict(name=name,kind=kind,status='passed',details=details))
        except FileNotFoundError as exc:
            missing=Path(exc.filename).name if exc.filename else 'required local input'
            results.append(dict(name=name,kind=kind,status='unavailable',error='Recorded replay fixture unavailable: '+missing))
        except Exception as exc:
            results.append(dict(name=name,kind=kind,status='failed',error=type(exc).__name__+': '+str(exc)))
    def stall():
        fixture=json.loads((FIXTURES/'native_zero_progress.json').read_text());rows=[r for r in fixture['rows'] if r['t']>=20]
        assert rows[-1]['t']-rows[0]['t']>=90
        span=math.hypot(max(r['x'] for r in rows)-min(r['x'] for r in rows),max(r['y'] for r in rows)-min(r['y'] for r in rows))
        assert span<.20 and all(r['state']=='running' and r['progress']==0 and r['localized'] for r in rows)
        samples=fixture['native_monitor_samples']
        assert all(s['planner_status']==1001 and s['path_points']>0 and not any(s['command']) for s in samples)
        return dict(recorded_running_s=round(rows[-1]['t']-rows[0]['t'],1),position_span_m=round(span,4),
                    current_fix_claimed=False,meaning='Historical running/zero-output failure confirmed; regression tests must validate handling.')
    def narrow():
        s=json.loads((FIXTURES/'narrow_forward_scan.json').read_text())['scan']
        forward=clearance(s,.35,0);assert forward[0],forward[1]
        explorer=Explorer(2);explorer.ingest(s);outcome=explorer.plan();assert outcome=='planned',outcome
        command=explorer.command();assert command[0]>0 and abs(command[1])<1e-9,command
        return dict(forward_feasible=True,command=list(command),physical_motion=False)
    def candidate():
        fixture=json.loads((FIXTURES/'conflicting_map_candidate.json').read_text())
        result=gate_alignment(fixture['metrics']);assert not result['allowed']
        return dict(publish_allowed=False,reason=result['reason'],physical_map_changed=False)
    def location():
        fixture=json.loads((FIXTURES/'localization_contract.json').read_text())
        localizer=AutomaticLocalizer.__new__(AutomaticLocalizer)
        localizer.app=SimpleNamespace(link=SimpleNamespace(record=lambda *a:dict(at=time.time(),items=dict(Location=0))))
        status=dict(localization_quality=dict(status=fixture['normal'],normal_status=fixture['normal'],age=.1),matching_quality=dict(error=.04,inlier_ratio=.99,age=.1))
        assert localizer.good(status)
        status['localization_quality']['status']=fixture['lost'];assert not localizer.good(status)
        return dict(normal=fixture['normal'],lost=fixture['lost'],scope='recorded installed firmware only')
    def cache():
        reader=RosSensors.__new__(RosSensors);reader.lock=threading.RLock();reader.ground_ready=threading.Condition(reader.lock)
        reader.autonomy_scan=dict(received_monotonic=time.monotonic()-.2,source_age=.1,stamp=123.)
        old=dict(reader.autonomy_scan);reader.ground_failure(TimeoutError('late next frame'),temporal=True)
        assert reader.autonomy_scan==old
        reader.autonomy_scan['received_monotonic']-=1;reader.ground_failure(TimeoutError('late next frame'),temporal=True)
        assert reader.autonomy_scan is None
        return dict(original_timestamp_retained=True,expired_cache_cleared=True)
    def transport_deadline():
        from platform_core import RobotLink
        from unittest.mock import patch
        for kind in ['expired_observation','wall_clock_skew']:
            link=RobotLink.__new__(RobotLink);link._stop=False;link.inhibit=False
            link.lock=threading.RLock();link.drive_lock=threading.RLock()
            link._vel=dict(f=.35,s=0.,y=0.);link._vel_ts=time.time()+3600;link._vel_last_nz=link._vel_ts
            link._vel_monotonic=time.monotonic()-(1 if kind=='wall_clock_skew' else 0);link._zero_until=0.
            link._observation_deadline=time.monotonic()-.01 if kind=='expired_observation' else None
            link.records={'1002:6':dict(at=time.time()+3600,monotonic_at=time.monotonic(),items=dict(BasicStatus=dict(HES=0,MotionState=17,Sleep=0,ControlUsageMode=0)))}
            emitted=[]
            def fake_axis(*axes):emitted.append(list(axes));link._stop=True
            link._send_axis=fake_axis;link._log=lambda *a:None
            with patch('platform_core.time.sleep'):link._vel_loop()
            assert emitted==[[0,0,0]] and not any(link._vel.values()),kind
        return dict(stale_moving_command_zeroed=True,wall_clock_skew_cannot_extend_motion=True,real_socket_created=False,hardware_stop_verified=False)
    check('recorded_native_zero_output',stall,'historical_failure_evidence')
    check('recorded_narrow_forward_passage',narrow)
    check('recorded_conflicting_candidate_rejected',candidate)
    check('installed_localization_contract',location)
    check('late_frame_cache_does_not_extend_deadline',cache)
    check('transport_deadline_without_next_request',transport_deadline)
    return results


def worker(args):
    blocked,suppressed=python_guard();sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
    if args._worker=='self-test':
        # Audit-only negative probes cannot open a socket or launch a child.
        for event in ['socket.connect','socket.sendto','subprocess.Popen']:
            try:sys.audit(event,None)
            except RuntimeError:pass
            else:raise AssertionError('effect guard not enforced: '+event)
        result=dict(status='passed',expected_denials=blocked,network_requests_sent=0,child_processes_started=0)
        print(MARKER+json.dumps(result));return 0
    if args._worker=='replay':
        rows=recorded_replay();result=dict(cases=rows,blocked_effects=blocked,suppressed_local_probes=suppressed)
        result['status']='failed' if blocked or any(r['status']=='failed' for r in rows) else 'passed' if all(r['status']=='passed' for r in rows) else 'unavailable'
        print(MARKER+json.dumps(result,ensure_ascii=False));return 0 if result['status']=='passed' else 2 if result['status']=='unavailable' else 1
    import unittest
    # Exercise the PRODUCT default, not merely the runner's offline variables.
    # If an optional model import attempts DNS, the audit hook and OS sandbox
    # stop it; that is a failed startup check, not an allowed test dependency.
    for key in ['YOLO_OFFLINE','YOLO_AUTOINSTALL']:os.environ.pop(key,None)
    try:
        import m20_console as original
        product_import=dict(offline_default=os.environ.get('YOLO_OFFLINE','').lower() in ['1','true','yes','on'],
                            autoinstall_disabled=os.environ.get('YOLO_AUTOINSTALL','').lower() in ['0','false','no','off'],
                            optional_model_library_available=original.YOLO is not None)
        product_import['passed']=product_import['offline_default'] and product_import['autoinstall_disabled'] and not blocked
    except Exception as exc:
        product_import=dict(passed=False,error=type(exc).__name__+': '+str(exc))
    finally:
        os.environ['YOLO_OFFLINE']='true';os.environ['YOLO_AUTOINSTALL']='false'
    if not product_import['passed']:
        print(MARKER+json.dumps(dict(status='failed',tests_run=0,passed=[],failed=[],errors=['product_import'],
                                    blocked_effects=blocked,product_import=product_import),ensure_ascii=False))
        return 1
    # Block native camera opens too; network sandboxing remains the final guard.
    try:
        import cv2
        def no_capture(*a,**kw):
            blocked.append('cv2.VideoCapture');raise RuntimeError('Offline acceptance forbids camera/capture opens')
        cv2.VideoCapture=no_capture
    except ImportError:pass
    class Result(unittest.TextTestResult):
        def __init__(self,*a,**kw):super().__init__(*a,**kw);self.passed=[]
        def addSuccess(self,test):super().addSuccess(test);self.passed.append(test.id())
    names=args.modules.split(',') if args.modules else []
    permitted={name for group in GROUPS.values() for name in group}
    if any(name not in permitted for name in names):raise ValueError('Test module is not allowlisted')
    suite=unittest.TestLoader().loadTestsFromNames(names)
    result=unittest.TextTestRunner(verbosity=1,resultclass=Result).run(suite)
    state='failed' if not result.wasSuccessful() or blocked else 'not_verified' if result.skipped else 'passed'
    payload=dict(tests_run=result.testsRun,passed=result.passed,failed=[t.id() for t,_ in result.failures],
                 errors=[t.id() for t,_ in result.errors],skipped=[dict(test=t.id(),reason=r) for t,r in result.skipped],
                 blocked_effects=blocked,suppressed_local_probes=suppressed,product_import=product_import,status=state)
    print(MARKER+json.dumps(payload,ensure_ascii=False));return 0 if state=='passed' else 2 if state=='not_verified' else 1


def parse_worker(run):
    for line in reversed(run.get('stdout','').splitlines()):
        if line.startswith(MARKER):
            result=json.loads(line[len(MARKER):]);result.update(returncode=run['returncode'],elapsed_s=run['elapsed_s'],os_network_denied=run['os_network_denied']);return result
    return dict(status='unavailable' if run['returncode'] is None else 'failed',returncode=run['returncode'],elapsed_s=run['elapsed_s'],os_network_denied=run['os_network_denied'],error=run['stderr'][-1800:])


def run_javascript():
    node=shutil.which('node')
    if not node:return dict(status='unavailable',error='Node.js unavailable'),None
    files=[ROOT/'tests'/name for name in JS_TESTS]
    missing=[p.name for p in files if not p.is_file()]
    if missing:return dict(status='unavailable',missing=missing),None
    command=[node,'--require',str(FIXTURES/'node_offline_guard.cjs'),'-e',
             'for (const file of process.argv.slice(1)) require(file)']+[str(p) for p in files]
    run=run_command(command,90);blocked=None
    for line in run['stderr'].splitlines():
        if line.startswith('OFFLINE_NODE_GUARD='):blocked=json.loads(line.split('=',1)[1])['blocked']
    count=lambda key: int(re.findall(r'^# '+key+r' (\d+)\s*$',run['stdout'],re.M)[-1]) if re.findall(r'^# '+key+r' (\d+)\s*$',run['stdout'],re.M) else None
    result=dict(returncode=run['returncode'],elapsed_s=run['elapsed_s'],os_network_denied=run['os_network_denied'],
                tests_run=count('tests'),passed=count('pass'),failed=count('fail'),blocked_effects=blocked,files=JS_TESTS)
    result['status']='unavailable' if run['returncode'] is None else 'passed' if run['returncode']==0 and blocked==[] and count('fail')==0 and count('tests') else 'failed'
    return result,run


def coverage(python_result,javascript_result):
    passed=python_result.get('passed',[]);rows=[]
    rows.append(dict(area='startup_without_internet',status='covered_offline' if python_result.get('product_import',{}).get('passed') else 'not_verified'))
    for name,tests in REQUIRED.items():
        missing=[test for test in tests if not any(t.endswith('.'+test) for t in passed)]
        rows.append(dict(area=name,status='covered_offline' if not missing else 'not_verified',missing_or_failed_tests=missing))
    nav=[t for t in passed if t.startswith('test_navigation_lifecycle.')]
    rows.append(dict(area='native_running_zero_output',status='covered_offline' if nav else 'not_verified',tests=nav))
    rows.append(dict(area='ui_wait_cancel_late_results',status='covered_offline' if javascript_result.get('status')=='passed' else 'not_verified'))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests',action='store_true',help='Run allowlisted mock tests with OS network denial')
    parser.add_argument('--suite',action='append',choices=sorted(GROUPS),help='Select suites; default is all')
    parser.add_argument('--record-baseline',action='store_true',help='Record source hashes only after ALL checks pass without source changes')
    parser.add_argument('--guard-self-test',action='store_true',help='Run non-I/O negative probes of the Python effect guard')
    parser.add_argument('--_worker',choices=['replay','tests','self-test'],help=argparse.SUPPRESS)
    parser.add_argument('--modules',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args._worker:return worker(args)
    if args.guard_self_test and args.tests:parser.error('--guard-self-test is a separate guard check, not a replacement for recorded replay')
    if args.record_baseline and (not args.tests or args.suite):parser.error('--record-baseline requires --tests with all suites')
    if args.suite and not args.tests:parser.error('--suite requires --tests')
    began=time.monotonic();before=source_inventory();observed_before=source_observation(before);runtime_before=runtime_inventory();OUTPUT.mkdir(parents=True,exist_ok=True)
    report=dict(schema_version=1,created_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
                mode='strict_offline',scope='computer source and local fixtures only',robot_state='NOT_CHECKED_OFFLINE',
                deployment_state='NOT_CHECKED_OFFLINE',physical_validation='NOT_PASSED_BY_THIS_TOOL',
                source_hashes=before,runtime_metadata=runtime_before,syntax_errors=syntax_check(before),evidence=evidence_manifest(),
                missing_replay_fixtures=[name for name in REPLAY_FIXTURES if not (FIXTURES/name).is_file()])
    missing_provenance=[s['path'] for f in report['evidence'] for s in f['sources'] if s.get('source_hash_matches') is False]
    report['changed_raw_evidence']=missing_provenance
    probe='self-test' if args.guard_self_test else 'replay'
    run=run_command([sys.executable,str(Path(__file__).resolve()),'--_worker',probe],30)
    report['guard_self_test' if args.guard_self_test else 'replay']=parse_worker(run)
    python_result=dict(status='not_run');js_result=dict(status='not_run');logs=[]
    if args.tests:
        groups=set(args.suite or GROUPS);modules=sorted({m for name in groups for m in GROUPS[name]})
        missing=[m for m in modules if not (ROOT/'tests'/(m.split('.',1)[0]+'.py')).is_file()]
        report['missing_test_modules']=missing
        with ThreadPoolExecutor(max_workers=2) as pool:
            py_future=pool.submit(run_command,[sys.executable,str(Path(__file__).resolve()),'--_worker','tests','--modules',','.join(m for m in modules if m not in missing)],180) if modules else None
            js_future=pool.submit(run_javascript) if 'ui' in groups else None
            if py_future:
                py_run=py_future.result();python_result=parse_worker(py_run);logs.append(('python',py_run))
            if js_future:
                js_result,js_run=js_future.result()
                if js_run:logs.append(('javascript',js_run))
        report.update(selected_suites=sorted(groups),python=python_result,javascript=js_result,coverage=coverage(python_result,js_result))
    after=source_inventory();observed_after=source_observation(after)
    report['source_changed_during_check']=sorted(set(changed(before,after)+changed(observed_before,observed_after)))
    runtime_after=runtime_inventory();report['runtime_changed_during_check']=runtime_before!=runtime_after
    checks=[not report['syntax_errors'],not missing_provenance,not report['source_changed_during_check'],not report['runtime_changed_during_check'],report.get('replay',report.get('guard_self_test',{})).get('status')=='passed']
    if args.tests:
        checks.extend([not report['missing_test_modules'],python_result.get('status')=='passed' if any(GROUPS[x] for x in groups) else True,js_result.get('status')=='passed' if 'ui' in groups else True])
    all_pass=all(checks)
    full=args.tests and not args.suite
    complete_coverage=full and all(row['status']=='covered_offline' for row in report['coverage'])
    if args.record_baseline and all_pass and complete_coverage:
        BASELINE.write_text(json.dumps(dict(schema_version=1,created_at=report['created_at'],source_hashes=after,runtime_metadata=runtime_after,python_tests=python_result.get('tests_run'),javascript_tests=js_result.get('tests_run')),ensure_ascii=False,indent=2)+'\n')
        report['baseline_recorded']=True
    previous=json.loads(BASELINE.read_text()) if BASELINE.is_file() else None
    delta=changed(previous['source_hashes'],after) if previous else None
    runtime_matches=previous is not None and previous.get('runtime_metadata')==runtime_after
    report['source_consistency']=dict(tested_baseline_available=previous is not None,changed_since_tested=delta,matches_tested_source=delta==[] if previous else False,runtime_metadata_matches_tested=runtime_matches)
    unavailable=report.get('replay',report.get('guard_self_test',{})).get('status')=='unavailable' or python_result.get('status')=='unavailable' or js_result.get('status')=='unavailable'
    failures=bool(report['syntax_errors'] or missing_provenance or report.get('replay',report.get('guard_self_test',{})).get('status')=='failed' or python_result.get('status')=='failed' or js_result.get('status')=='failed')
    if report['source_changed_during_check'] or report['runtime_changed_during_check']:status='SOURCE_CHANGED_DURING_CHECK';exit_code=2
    elif failures:status='FAILED';exit_code=1
    elif unavailable or not all_pass:status='NOT_FULLY_VERIFIED';exit_code=2
    elif args.guard_self_test and not args.tests:status='GUARD_VERIFIED';exit_code=0
    elif args.tests and (not full or complete_coverage):status='OFFLINE_TESTS_PASSED';exit_code=0
    elif not args.tests and delta==[] and runtime_matches:status='QUICK_CHECK_PASSED';exit_code=0
    else:status='NOT_FULLY_VERIFIED';exit_code=2
    report.update(status=status,elapsed_s=round(time.monotonic()-began,3))
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d-%H%M%S-%f')
    dest=OUTPUT/(stamp+('-tests' if args.tests else '-quick')+'.json')
    dest.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(OUTPUT/'latest.json').write_text(dest.read_text())
    for name,log in logs:(OUTPUT/(stamp+'-'+name+'.log')).write_text(log['stdout']+'\n'+log['stderr'])
    print(json.dumps(dict(status=status,elapsed_s=report['elapsed_s'],python_tests=python_result.get('tests_run'),javascript_tests=js_result.get('tests_run'),
                         matches_tested_source=report['source_consistency']['matches_tested_source'],source_changed_during_check=report['source_changed_during_check'],
                         report=str(dest),physical_validation=report['physical_validation']),ensure_ascii=False,indent=2))
    return exit_code


if __name__=='__main__':sys.exit(main())
