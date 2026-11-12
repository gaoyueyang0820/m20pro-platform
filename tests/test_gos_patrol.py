"""Synthetic controls, map files and station adapters. No real robot or sockets."""
import math
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from bridge.gos_agent import GOSAgent
from bridge.gos_navigation import GOSNavigation, pack_grid
from bridge.map_service import MapService
from bridge.resident_mission import held_map_lease, manage_map_lease, navigation_grid_identity
from bridge.root_actions import execute
from bridge.task_collection import CollectionCancelled, RouteCollection
from m20.autonomy_core import disk
from m20.patrol_actions import FastBee, StationActions, station_spec
from m20.point_navigation import PointNavigator
from m20.resident_navigation_client import ResidentNavigationClient
from m20.task_collectors import IoTSampler, collector_specs


class LocalLink:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = [2., 3., 0.]
        self.axes = [0., 0., 0.]
        self.deadline = 0.
        self.stamp = time.monotonic()
        self.events = []
        self.auxiliary = False
        self.located = True

    def _advance(self):
        now = time.monotonic()
        elapsed = max(0., min(now, self.deadline) - self.stamp)
        f, s, yaw = self.axes
        c, d = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += (c*f-d*s)*elapsed
        self.pose[1] += (d*f+c*s)*elapsed
        self.pose[2] += yaw*elapsed
        self.stamp = now
        if now >= self.deadline:
            self.axes = [0., 0., 0.]

    def record(self, kind, command):
        with self.lock:
            self._advance()
            records = {
                (1002, 6): dict(BasicStatus=dict(HES=0, MotionState=17, Sleep=False,
                                                ControlUsageMode=2 if self.auxiliary else 0, Gait=0x3002, Direction=0,
                                                OOA=3 if self.auxiliary else 0)),
                (1002, 4): dict(MotionStatus=dict(zip(('LinearX','LinearY','OmegaZ'),self.axes))),
                (1007, 1): dict(Status=0),
                (1007, 2): dict(Location=0 if self.located else 1, Confidence=.99, **dict(zip(('PosX','PosY','Yaw'),self.pose))),
                (2002, 1): dict(ObsState=1 if self.auxiliary else 0),
            }
            return dict(items=records[(kind,command)], monotonic_at=self.stamp)

    def set_velocity(self, f, s, yaw, valid_until=None):
        with self.lock:
            self._advance()
            self.axes = [f, s, yaw]
            self.deadline = valid_until or self.stamp
            self.events.append((self.stamp, tuple(self.axes)))

    def close(self, stop=True):
        if stop:
            self.set_velocity(0., 0., 0.)


class LocalScans:
    def __init__(self, link):
        self.link = link
        self.stamps = []

    def call(self, path, request, timeout):
        time.sleep(.035)
        record = self.link.record(1007, 2)
        stamp = record['monotonic_at']
        self.stamps.append((request.get('after_stamp'), stamp))
        pose = [record['items'][k] for k in ('PosX','PosY','Yaw')]
        floor = list(disk(2.2))
        return dict(ok=True, protocol=1, frame_id='base_link', pose_frame_id='map',
                    pose_stamp=stamp, stamp=stamp, resolution=.2, pose=pose,
                    age=0., server_elapsed_s=.035, floor=floor, measured_floor=floor, obstacles=[])


class GOSPatrolTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.data = self.root/'runtime'
        self.maps = self.root/'maps'
        folder = self.maps/'synthetic'
        folder.mkdir(parents=True)
        (self.maps/'active').symlink_to(folder.name, target_is_directory=True)
        (self.maps/'.m20-catalog.lock').touch()
        self.raw = b'P5\n64 64\n255\n' + bytes([254])*64*64
        (folder/'occ_grid.pgm').write_bytes(self.raw)
        (folder/'occ_grid.yaml').write_text('resolution: 0.1\norigin: [0,0,0]\n')
        self.map_service = MapService(self.maps, self.data, demo=True)
        grid = self.map_service.grid()
        self.map_identity = navigation_grid_identity(grid, self.raw)
        self.request = dict(id='1'*32, owner='2'*32, map=self.map_identity, grid=pack_grid(self.raw, grid),
                            points=[dict(x=2.55,y=3.,yaw=0.),dict(x=3.12,y=3.,yaw=0.)])
        self.identity = {k:self.request[k] for k in ('id','owner')}
        self.link = LocalLink()
        self.sensors = LocalScans(self.link)
        self.actions = Mock()
        self.actions.capabilities.return_value = dict(people_count=True, ball_camera=True, ptz=True, iot=True)
        self.factory = Mock(return_value=self.link)
        self.service = GOSNavigation(self.data, lambda:self.sensors, aos='10.1.2.3', enabled=True,
                                     link_factory=self.factory, actions=self.actions)
        self.addCleanup(self.service.close)

    def wait_for(self, condition, timeout=12, pulse=False):
        deadline = time.monotonic() + timeout
        next_pulse = time.monotonic() + 1
        while time.monotonic() < deadline:
            status = self.service.status()
            if condition(status):
                return status
            if pulse and time.monotonic() >= next_pulse:
                self.service.dispatch('/missions/pulse', self.identity)
                next_pulse = time.monotonic() + 1
            time.sleep(.02)
        self.fail(self.service.status())

    def test_preview_is_read_only(self):
        result = self.service.dispatch('/missions/preview', dict(grid=self.request['grid'],start=[2.,3.,0.],
                                                                goal=[3.,3.],yaw=0.))
        self.assertGreater(result['plan']['length_m'], .8)
        self.factory.assert_not_called()

    def test_temporary_localization_loss_stops_axes_then_resumes_same_job(self):
        self.service.start(self.request)
        self.wait_for(lambda _:any(any(axes) for _,axes in self.link.events),timeout=5,pulse=True)
        self.link.located=False
        self.wait_for(lambda s:'定位恢复' in s['message'],timeout=2,pulse=True)
        parked=len(self.link.events)
        began=time.monotonic()
        status=self.wait_for(lambda _:time.monotonic()-began>5.1,timeout=6,pulse=True)
        self.assertEqual(status['phase'],'running')
        self.assertTrue(all(not any(axes) for _,axes in self.link.events[parked:]))
        self.link.located=True
        result=self.wait_for(lambda s:s.get('stop_confirmed'),pulse=True)
        self.assertEqual((result['phase'],result['completed']),('completed',2))

    def test_control_preparation_waits_for_the_first_basic_reply(self):
        navigator=object.__new__(PointNavigator)
        navigator.app=SimpleNamespace(lock=threading.RLock(),link=self.link)
        navigator.check=Mock()
        record=self.link.record
        basic_reads=0
        def delayed(kind,command):
            nonlocal basic_reads
            if (kind,command)==(1002,6):
                basic_reads+=1
                if basic_reads==1:return {}
            return record(kind,command)
        with patch.object(self.link,'record',side_effect=delayed):
            navigator._prepare_motion('synthetic')
        self.assertEqual(basic_reads,2)
        self.assertEqual(navigator.motion_mode,0)
        self.assertEqual(self.link.events,[])

    def test_device_only_station_reads_each_device_while_stopped(self):
        actions = StationActions(self.service.aos, dict(iot_url='http://gateway.example.invalid',iot_token='fixture'))
        def read(device):
            self.assertEqual(tuple(self.link.axes),(0.,0.,0.))
            if device=='43':raise ValueError('设备离线')
            return {'温度':24.5}
        actions.fastbee.telemetry=Mock(side_effect=read)
        self.service.actions=actions
        self.request['points'][0]['station_action']=dict(camera='none',iot_device_ids=['42','43'])
        with patch.object(actions,'_counts',side_effect=AssertionError('设备任务不应打开相机')):
            self.service.start(self.request)
            result=self.wait_for(lambda row:row['phase']=='completed',pulse=True)
        station=result['station_results'][0]
        self.assertNotIn('people',station)
        self.assertEqual(station['devices'][0]['measurements'],{'温度':24.5})
        self.assertEqual(station['devices'][1]['error'],'设备离线')
        self.assertEqual(result['completed'],2)

    def test_iot_catalog_configuration_and_collection_share_the_supported_contract(self):
        agent=GOSAgent(self.data)
        self.addCleanup(agent.close)
        seen=[]
        large='9007199254740993'
        def endpoint(client,path,method='GET',data=None,token=None,timeout=3):
            seen.append(path)
            if path.startswith('/captchaImage'):
                return 200,dict(code=200,captchaEnabled=True,uuid='fixture-captcha',img='aW1hZ2U=')
            if path=='/login':
                self.assertEqual(data['code'],'9')
                return 200,dict(code=200,token='fixture-iot-token')
            self.assertEqual(token,'fixture-iot-token')
            if path.startswith('/iot/device/list'):
                return 200,dict(code=200,rows=[dict(deviceId=int(large),deviceName='气体检测',status=3)],total=21)
            if path=='/iot/device/runningStatus?deviceId='+large:
                return 200,dict(code=200,data=dict(thingsModelValue=None,serialNumber='gas-demo'))
            if path=='/iot/runtime/running?serialNumber=gas-demo':
                return 200,dict(code=200,data=[dict(id='methane',name='甲烷',value='2.5',datatype=dict(unit='%LEL'))])
            if path=='/iot/device/runningStatus?deviceId=43':
                return 403,{}
            raise AssertionError(path)
        with patch.object(FastBee,'_json',endpoint):
            params=dict(url='http://gateway.example.invalid',user='fixture-user',password='fixture-secret')
            response=agent.dispatch('/iot/settings',params)
            self.assertIn('challenge',response)
            self.assertFalse(agent.patrol_config.exists())
            response=agent.dispatch('/iot/settings',dict(params,code='9',uuid=response['challenge']['uuid']))
            self.assertTrue(response['configured'])
            self.assertEqual(response['devices'][0]['id'],large)
            self.assertNotIn('fixture-iot-token',str(response))
            self.assertNotIn('password',agent.dispatch('/iot/settings',None))
            page=agent.dispatch('/iot/devices',dict(page=2,name='气体'))
            self.assertFalse(page['has_more'])
            self.assertTrue(any('pageNum=2' in p and 'deviceName=' in p for p in seen))
            spec,=collector_specs([dict(kind='iot_readings',device_ids=[large,'43'],interval_s=5)])
            sample=IoTSampler(SimpleNamespace(fastbee=agent.iot_client()),spec).sample()
        self.assertNotIn('error',sample)
        self.assertEqual(sample['devices'][0]['measurements'],{'甲烷 (%LEL)':'2.5'})
        self.assertIn('403',sample['devices'][1]['error'])

    def test_station_parks_then_counts_then_moves_to_next_point(self):
        self.request['points'][0]['station_action'] = dict(camera='ball',pan='right',iot_device_id='42')
        during = []
        def inspect(action, guard):
            guard()
            during.append((self.link.pose[0], tuple(self.link.axes), self.service.status()['stage']))
            self.assertEqual(action['iot_device_id'], '42')
            return dict(camera='ball',pan='right',people=3,samples=[2,3,3])
        self.actions.run.side_effect = inspect
        self.service.dispatch('/missions/start', self.request)
        result = self.wait_for(lambda s:s['stop_confirmed'], pulse=True)
        self.assertEqual(result['phase'], 'completed', result)
        self.assertEqual(result['completed'], 2)
        self.assertEqual(len(during), 1)
        self.assertGreater(during[0][0], 2.32)
        self.assertEqual(during[0][1:], ((0.,0.,0.),'station_action'))
        self.assertGreater(self.link.pose[0], 2.9)
        self.assertEqual(result['station_results'][0]['people'], 3)
        self.assertTrue(any(after is not None for after,_ in self.sensors.stamps))

    def test_along_route_counts_while_moving_and_slow_iot_does_not_hold_navigation(self):
        started = threading.Event()
        walking = []
        self.link.auxiliary = True
        self.request['collectors'] = [dict(kind='people_count',camera='front',interval_s=1),
                                      dict(kind='iot_readings',device_id='42',interval_s=2)]
        def worker_factory(data, aos, spec):
            def sample(cancelled):
                if spec['kind']=='iot_readings':
                    started.set()
                    while not cancelled():
                        time.sleep(.03)
                    raise CollectionCancelled
                walking.append((self.link.pose[0], tuple(self.link.axes)))
                return dict(sampled_at=time.time(),values={'people':3})
            return SimpleNamespace(sample=sample,close=lambda:None)
        def collection(*args):
            return RouteCollection(*args,worker_factory=worker_factory)
        with patch('bridge.gos_navigation.RouteCollection',side_effect=collection):
            self.service.dispatch('/missions/start',self.request)
        result=self.wait_for(lambda s:s.get('stop_confirmed'),pulse=True)
        self.assertEqual(result['phase'],'completed',result)
        self.assertTrue(started.is_set())
        self.assertTrue(self.service.nav['vendor_avoidance'])
        for worker in self.service.collection.threads:
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
        self.assertTrue(any(2. <= x < 3.12 and any(axes) for x,axes in walking),walking)
        rows=self.service.dispatch('/missions/collection',dict(id=self.identity['id'],offset=0))['rows']
        self.assertTrue(rows)
        self.assertTrue(all(row['kind']=='people_count' and row['values']=={'people':3} for row in rows))
        self.assertEqual(result['collection'][0]['peak_people'],3)
        self.assertEqual(result['collection'][1]['samples'],0)

    def test_expired_lease_parks_and_late_pulse_does_not_resume(self):
        self.request['points'][0]['x'] = 4.
        self.service.dispatch('/missions/start', self.request)
        self.wait_for(lambda _:any(any(axes) for _,axes in self.link.events),timeout=5)
        with self.service.lock:
            self.service.lease_until = time.monotonic() - .1
        self.service.dispatch('/missions/pulse', self.identity)
        result = self.wait_for(lambda s:s['stop_confirmed'],timeout=3)
        self.assertEqual(result['phase'],'cancelled')
        self.assertEqual(self.link.events[-1][1],(0.,0.,0.))
        index = len(self.link.events)
        self.service.dispatch('/missions/pulse', self.identity)
        time.sleep(.08)
        self.assertFalse(any(any(axes) for _,axes in self.link.events[index:]))

    def test_desktop_delegates_route_and_records_station_result_after_stop(self):
        cache = self.root/'desktop'
        (cache/'maps').mkdir(parents=True)
        (cache/'maps/map.pgm').write_bytes(self.raw)
        connections = SimpleNamespace(
            value=dict(aos_identity_alias='synthetic', targets={role:dict(enabled=True) for role in ('nos','gos')}),
            endpoint=lambda role:(role,'test-token'))
        app = SimpleNamespace(store=SimpleNamespace(path=cache/'workspace.json'),lock=threading.RLock(),
                              hub=SimpleNamespace(connections=connections),demo=False,link=self.link,
                              nav=dict(status='idle'),_record_route_completion=Mock(),
                              maps=lambda:[dict(id='map',robot_map_name='synthetic',grid_id=0,resolution=.1,origin=[0.,0.,0.])])
        client = ResidentNavigationClient(app)
        self.addCleanup(client.close)
        actions = dict(camera='front',pan='none',iot_device_id='42')
        self.actions.run.return_value = dict(camera='front',pan='none',people=2,samples=[1,2,2])
        nav = dict(status='running',disconnect_policy='stop',
                   options=dict(NavMode=1,Gait=0x3002,Speed=1,ObsMode=0,Manner=0),
                   route=dict(map_id='map',points=[dict(x=2.,y=3.,yaw=0.,station_action=actions)]))
        seen = []
        def rpc(role,path,req,timeout,token):
            seen.append((role,path,(req or {}).get('action')))
            if role=='nos' and path=='/maps/grid':
                return self.map_service.grid()
            if role=='nos' and path=='/maps/navigation-lease':
                return manage_map_lease(self.map_service,req)
            self.assertEqual(role,'gos')
            if path=='/sensors/status':return dict(odom=dict(frame_id='map'))
            return self.service.dispatch(path,req)
        with patch('m20.resident_navigation_client.rpc',side_effect=rpc):
            client.prepared(nav,executor='gos')
            deadline=time.monotonic()+7
            while client.busy() and time.monotonic()<deadline:
                time.sleep(.03)
            self.assertFalse(client.busy(), nav)
        self.assertEqual(nav['status'],'completed')
        self.assertEqual(nav['station_results'][0]['people'],2)
        self.assertLess(seen.index(('nos','/maps/navigation-lease','claim')),
                        seen.index(('gos','/missions/start',None)))
        self.assertIn(('nos','/maps/navigation-lease','release'),seen)
        app._record_route_completion.assert_called_once()

    def test_map_lease_excludes_mutation_and_detects_changed_grid(self):
        claim = dict(self.identity, action='claim', map=self.map_identity)
        manage_map_lease(self.map_service, claim)
        self.assertIsNotNone(held_map_lease(self.data))
        with self.assertRaisesRegex(ValueError,'GOS 导航正在使用'):
            execute({'action':'localize_auto'}, maps=self.maps, data=self.data)
        (self.maps/'active'/'occ_grid.pgm').write_bytes(self.raw+b' ')
        with self.assertRaisesRegex(ValueError,'授权已变化'):
            manage_map_lease(self.map_service,dict(self.identity,action='pulse'))
        manage_map_lease(self.map_service,dict(self.identity,action='release'))
        self.assertIsNone(held_map_lease(self.data))


class PatrolIntegrationTests(unittest.TestCase):
    def test_gos_navigation_can_read_paired_nos_scans_over_loopback(self):
        with tempfile.TemporaryDirectory() as directory:
            data=Path(directory)
            (data/'gos-nav-sensors.local.json').write_text(json.dumps(dict(port=8095,token='test-only')))
            remote=Mock()
            remote.status.return_value=dict(odom=dict(frame_id='map'))
            remote.autonomy.return_value=dict(pose=[0,0,0])
            with patch('bridge.sensor_client.SensorClient',return_value=remote) as factory:
                agent=GOSAgent(data)
                self.assertEqual(agent.dispatch('/sensors/status',None)['navigation_sensor_host'],'nos')
                self.assertEqual(agent.dispatch('/sensors/autonomy',dict(mapping=False))['pose'],[0,0,0])
            factory.assert_called_once_with(port=8095,token='test-only')

    def test_fastbee_direction_and_stop_follow_sip_v2_contract(self):
        bridge = FastBee(dict(iot_url='http://iot.example.invalid/prod-api',iot_token='fake',
                              sip_device='1'*20,sip_channel='2'*20,fastbee_ptz=True))
        bridge._json = Mock(return_value=(200,dict(code=200,data=True)))
        bridge.ptz('right')
        bridge.ptz('stop')
        args = [call.args for call in bridge._json.call_args_list]
        self.assertTrue(args[0][0].endswith('/RIGHT/35'))
        self.assertTrue(args[1][0].endswith('/STOP'))
        self.assertEqual([row[1] for row in args], ['POST','POST'])

    def test_camera_move_timeout_still_issues_stop_and_skips_counting(self):
        station = StationActions('10.1.2.3')
        station.fastbee.ptz = Mock(side_effect=[ValueError('timed out'), None])
        station._counts = Mock()
        with self.assertRaisesRegex(ValueError,'timed out'):
            station.run(station_spec(dict(camera='ball',pan='left')),lambda:None)
        self.assertEqual([call.args[0] for call in station.fastbee.ptz.call_args_list], ['left','stop'])
        station._counts.assert_not_called()
