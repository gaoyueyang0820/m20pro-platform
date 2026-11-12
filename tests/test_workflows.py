"""Offline integration tests for v2 workflows. Never contacts a robot or GitHub."""
import base64
import copy
import io
import json
import math
from pathlib import Path
import stat
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch,MagicMock
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'bridge'))
import m20_platform as web
from m20.connection_manager import Connections,target_config,host
from m20.map_formats import Files,CHUNK,digest,inspect_package,pack_map,unpack_package,preview_pcd,lzf_decode
from m20.platform_core import Store,JOINTS,NEUTRAL,leg_forward,leg_target
from bridge.map_service import MapService
from bridge import root_actions,mapping_job
from bridge.install_remote import ROOT_SETUP
from sdk_agent import SDKAgent,RosPolicy
from bridge.ros_sensors import point_cloud
from bridge.gos_agent import GOSAgent


def make_map(root,name='warehouse'):
    folder=Path(root)/name;folder.mkdir(parents=True)
    (folder/'occ_grid.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xff\x00')
    (folder/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n')
    (folder/'full_cloud.pcd').write_bytes(pcd_header('ascii',3)+b'1 2 3\n4 5 6\n7 8 9\n')
    (folder/'occ_grid_3.pgm').write_bytes((folder/'occ_grid.pgm').read_bytes())
    (folder/'occ_grid_3.yaml').write_text((folder/'occ_grid.yaml').read_text())
    (folder/'occ_grid_id_map.toml').write_text('# Original vendor mapping, schema preserved\n')
    (folder/'blocks').mkdir();(folder/'blocks/metadata.bin').write_bytes(b'opaque-localization-data')
    return folder


def pcd_header(encoding,total):
    return ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %s\nHEIGHT 1\nPOINTS %s\nDATA %s\n'%(total,total,encoding)).encode()


def await_job(app,response):
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        job=app.hub.operations.get(response['job_id'])
        if job['status']=='failed':raise AssertionError(job['error'])
        if job['status']=='completed':return job['result']
        time.sleep(.01)
    raise AssertionError('Offline job did not finish')


class MapFilesTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()
    def test_resume_order_and_hash(self):
        files=Files(self.root/'files');raw=pcd_header('ascii',1)+b'1 2 3\n'
        req=dict(name='cloud.pcd',size=len(raw),kind='pcd');m=files.start(req)['file']
        files.chunk(dict(id=m['id'],offset=0,data=base64.b64encode(raw[:12]).decode()))
        resumed=Files(files.root).start(dict(req,resume_id=m['id']))['file']
        self.assertEqual(resumed['offset'],12)
        with self.assertRaises(ValueError):files.chunk(dict(id=m['id'],offset=0,data='YQ=='))
        files.chunk(dict(id=m['id'],offset=12,data=base64.b64encode(raw[12:]).decode()))
        with self.assertRaises(ValueError):files.finish(dict(id=m['id'],sha256='incorrect'))
        completed=files.finish(dict(id=m['id']))['file']
        self.assertEqual(completed['sha256'],digest(files.path(m['id'])))
        self.assertEqual(base64.b64decode(files.read_chunk(m['id'],0)['data']),raw)
        with self.assertRaises(ValueError):files.start(dict(req,resume_id='../file'))
    def test_full_directory_roundtrip_and_grid_id(self):
        folder=make_map(self.root/'maps');archive=self.root/'warehouse.zip';pack_map(folder,archive)
        inspected=inspect_package(archive);self.assertEqual(inspected['grid_ids'],[0,3])
        out=self.root/'imported';unpack_package(archive,out,'warehouse-copy')
        copy=out/'warehouse-copy'
        self.assertEqual((copy/'blocks/metadata.bin').read_bytes(),b'opaque-localization-data')
        self.assertTrue((copy/'occ_grid_id_map.toml').exists())
        self.assertIn(str(copy/'occ_grid_3.pgm'),(copy/'occ_grid_3.yaml').read_text())
        with self.assertRaises(ValueError):unpack_package(archive,out,'warehouse-copy')
        with self.assertRaises(ValueError):unpack_package(archive,out,'active')
        service=MapService(out,self.root/'data',True)
        self.assertEqual(service.library()['maps'][0]['grids'],[0,3])
        self.assertEqual(service.grid('warehouse-copy',3)['grid_id'],3)
    def test_unsafe_archive_is_rejected_before_extraction(self):
        folder=make_map(self.root/'maps');good=self.root/'good.zip';pack_map(folder,good)
        for name,mode in [('../escape',stat.S_IFREG),('/absolute',stat.S_IFREG),('warehouse/link',stat.S_IFLNK),('warehouse\\bad',stat.S_IFREG)]:
            target=self.root/'bad.zip';target.write_bytes(good.read_bytes())
            with zipfile.ZipFile(target,'a') as z:
                info=zipfile.ZipInfo(name);info.external_attr=(mode|0o777)<<16;z.writestr(info,'payload')
            with self.assertRaises(ValueError):inspect_package(target)
            self.assertFalse((self.root/'escape').exists())
    def test_real_import_and_apply_delegate_to_vendor(self):
        maps=self.root/'maps';folder=make_map(maps)
        archive=self.root/'warehouse.zip';pack_map(folder,archive)
        service=MapService(maps,self.root/'data')
        meta=service.files.register(archive)
        with patch.object(service,'privileged',return_value={'ok':True}) as call:
            service.dispatch('/maps/package/import',dict(id=meta['id'],name='warehouse'))
            self.assertEqual(call.call_args.args[0],'import')
        with patch.object(root_actions,'run',return_value='ok') as run:
            root_actions.execute(dict(action='apply',name='warehouse'),maps,self.root/'data')
            self.assertEqual(run.call_args_list[0].args[0],['drmap','apply',str(folder.resolve())])
        self.assertFalse((maps/'active').exists())
    def test_real_import_refuses_sibling_files_before_vendor_unpack(self):
        folder=make_map(self.root/'original');archive=self.root/'warehouse.zip';pack_map(folder,archive)
        with zipfile.ZipFile(archive,'a') as z:z.writestr('existing-map/do-not-overwrite.bin',b'bad')
        data=self.root/'data';files=Files(data/'files');meta=files.register(archive)
        maps=self.root/'destination';maps.mkdir()
        with patch.object(root_actions,'run') as run:
            with self.assertRaises(ValueError):root_actions.execute(dict(action='import',id=meta['id'],name='warehouse',sha256=meta['sha256']),maps,data)
            run.assert_not_called()
    def test_map_paths_never_escape_library(self):
        maps=self.root/'maps';make_map(maps);outside=self.root/'outside';outside.mkdir()
        (maps/'active').symlink_to(outside)
        service=MapService(maps,self.root/'data',True)
        with self.assertRaises(ValueError):service.grid('active')
        with self.assertRaises(ValueError):service.path('../outside')
    def test_vendor_internal_directory_link_is_materialized_and_roundtrips(self):
        folder=make_map(self.root/'maps');nav=folder/'.nav_maps/nav_map_0';nav.mkdir(parents=True)
        (nav/'navigation.bin').write_bytes(b'vendor navigation payload')
        (folder/'.nav_maps/active').symlink_to(nav,target_is_directory=True)
        (folder/'cloud-copy.pcd').symlink_to('full_cloud.pcd')
        archive=self.root/'warehouse.zip';result=pack_map(folder,archive)
        self.assertEqual(result['materialized_links'],2)
        with zipfile.ZipFile(archive) as z:
            self.assertEqual(z.read('warehouse/.nav_maps/active/navigation.bin'),b'vendor navigation payload')
            self.assertEqual(z.read('warehouse/cloud-copy.pcd'),(folder/'full_cloud.pcd').read_bytes())
            self.assertEqual(stat.S_IMODE(z.getinfo('warehouse/full_cloud.pcd').external_attr>>16),0o644)
            self.assertFalse(any(stat.S_ISLNK(i.external_attr>>16) for i in z.infolist()))
        unpack_package(archive,self.root/'imported','copy')
        self.assertFalse((self.root/'imported/copy/.nav_maps/active').is_symlink())
        self.assertEqual((self.root/'imported/copy/.nav_maps/active/navigation.bin').read_bytes(),b'vendor navigation payload')
    def test_map_export_refuses_escape_and_keeps_existing_output(self):
        folder=make_map(self.root/'maps');outside=self.root/'private.txt';outside.write_text('must not export')
        alias=folder/'bad-link';alias.symlink_to(outside)
        output=self.root/'map.zip';output.write_bytes(b'previous export')
        with self.assertRaisesRegex(ValueError,'超出当前地图目录'):pack_map(folder,output)
        self.assertEqual(output.read_bytes(),b'previous export')
        alias.unlink();alias.symlink_to(self.root,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'超出当前地图目录'):pack_map(folder,output)
        self.assertEqual(output.read_bytes(),b'previous export')
    def test_map_export_rejects_broken_cycle_and_output_inside_map(self):
        folder=make_map(self.root/'maps');alias=folder/'bad-link';output=self.root/'map.zip'
        alias.symlink_to('missing')
        with self.assertRaisesRegex(ValueError,'失效或循环'):pack_map(folder,output)
        alias.unlink();alias.symlink_to(folder,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'循环'):pack_map(folder,output)
        alias.unlink()
        with self.assertRaisesRegex(ValueError,'输出不能位于'):pack_map(folder,folder/'map.zip')
    def test_vendor_export_uses_explicit_destination_and_exact_map_name(self):
        maps=self.root/'maps';folder=make_map(maps);(maps/'active').symlink_to(folder.name)
        calls=[]
        def runner(argv,timeout):
            calls.append(argv)
            self.assertEqual(argv[:3],['drmap','pack','-d']);self.assertEqual(argv[4],str(folder.resolve()))
            pack_map(folder,Path(argv[3])/(folder.name+'.zip'))
            return 'vendor complete'
        service=MapService(maps,self.root/'agent',False,runner)
        result=service.dispatch('/maps/package/export',dict(name='warehouse'))
        self.assertEqual(result['format'],'drmap');self.assertEqual(len(calls),1)
        self.assertTrue(service.files.path(result['file']['id']).is_file())
        self.assertFalse(Path(calls[0][3]).exists())
    def test_real_import_explicitly_disables_vendor_automatic_activation(self):
        folder=make_map(self.root/'source');archive=self.root/'warehouse.zip';pack_map(folder,archive)
        data=self.root/'agent';files=Files(data/'files');meta=files.register(archive)
        maps=self.root/'destination';maps.mkdir()
        def unpack(argv):
            self.assertEqual(argv[:3],['drmap','unpack','-n'])
            self.assertEqual(Path(argv[3]).name,'warehouse.zip')
            (maps/'warehouse').mkdir();return 'unpacked without activating'
        with patch.object(root_actions,'run',side_effect=unpack) as run:
            result=root_actions.execute(dict(action='import',id=meta['id'],name='warehouse',sha256=meta['sha256']),maps,data)
            self.assertFalse(result['activated']);self.assertEqual(run.call_count,1)


class PCDTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'cloud.pcd'
    def tearDown(self):self.tmp.cleanup()
    def test_binary_and_field_major_compressed_agree(self):
        points=[(1.,2.,3.),(-4.,5.,6.),(7.,8.,9.)]
        self.path.write_bytes(pcd_header('binary',3)+b''.join(struct.pack('<fff',*p) for p in points))
        plain=preview_pcd(self.path)
        raw=b''.join(struct.pack('<f',p[axis]) for axis in range(3) for p in points)
        compressed=b''.join(bytes([len(raw[i:i+32])-1])+raw[i:i+32] for i in range(0,len(raw),32))
        self.path.write_bytes(pcd_header('binary_compressed',3)+struct.pack('<II',len(compressed),len(raw))+compressed)
        self.assertEqual(preview_pcd(self.path)['points'],plain['points'])
        self.assertEqual(plain['bounds'],[[-4.,2.,3.],[7.,8.,9.]])
    def test_ascii_filters_invalid_and_bounds_sample_size(self):
        self.path.write_bytes(pcd_header('ascii',3)+b'nan 0 0\n1 2 3\n4 inf 5\n')
        self.assertEqual(preview_pcd(self.path)['points'],[[1.,2.,3.]])
        self.path.write_bytes(pcd_header('ascii',1000)+b'1 2 3\n'*1000)
        self.assertEqual(preview_pcd(self.path,100)['shown_points'],100)
    def test_truncated_binary_and_lzf_refuse(self):
        self.path.write_bytes(pcd_header('binary',3)+b'\x00'*8)
        with self.assertRaises(ValueError):preview_pcd(self.path)
        for raw,expected in [(b'\x01a',2),(b'\x20\x00',3),(b'\x00a',4)]:
            with self.assertRaises(ValueError):lzf_decode(raw,expected)
        self.assertEqual(lzf_decode(b'\x00a\x60\x00',6),b'aaaaaa')
    def test_ros_cloud_respects_row_padding_and_endianness(self):
        fields=[SimpleNamespace(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate(['x','y','z'])]
        stamp=SimpleNamespace(sec=1,nanosec=2)
        for endian in ['<','>']:
            data=struct.pack(endian+'fff',1,2,3)+b'PAD!'+struct.pack(endian+'fff',4,5,6)+b'PAD!'
            message=SimpleNamespace(fields=fields,width=1,height=2,point_step=12,row_step=16,data=data,is_bigendian=endian=='>',
                                    header=SimpleNamespace(frame_id='map',stamp=stamp))
            self.assertEqual(point_cloud(message)['points'],[[1.,2.,3.],[4.,5.,6.]])
        message.row_step=8
        with self.assertRaises(ValueError):point_cloud(message)


class ConnectionTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.connections=Connections(self.tmp.name)
    def tearDown(self):self.connections.close();self.tmp.cleanup()
    def test_profile_persists_without_exposing_tokens(self):
        c=self.connections
        c.save(dict(auto_connect=True,targets=dict(nos=dict(enabled=True,host='10.21.31.106'))))
        public=json.dumps(c.public());self.assertNotIn(c.value['targets']['nos']['token'],public)
        self.assertNotIn('token',c.public()['targets']['nos'])
        self.assertEqual(c.path.stat().st_mode&0o777,0o600)
        self.assertTrue(Connections(self.tmp.name).value['auto_connect'])
    def test_legacy_profile_keeps_sdk_endpoint_and_adds_optional_gos(self):
        c=self.connections
        c.save(dict(targets=dict(sdk=dict(host='10.21.31.103',domain_id=7))))
        saved=json.loads(c.path.read_text());saved['targets'].pop('gos');saved.pop('sensor_source')
        c.path.write_text(json.dumps(saved))
        restored=Connections(self.tmp.name)
        self.assertEqual(restored.value['targets']['sdk']['host'],'10.21.31.103')
        self.assertEqual(restored.value['targets']['sdk']['domain_id'],7)
        self.assertEqual(restored.value['targets']['gos']['host'],'10.21.31.104')
        self.assertFalse(restored.value['targets']['gos']['enabled'])
        self.assertEqual(restored.value['sensor_source'],'nos')
    def test_nos_and_gos_tunnels_coexist_and_disconnect_independently(self):
        c=self.connections
        c.save(dict(targets=dict(nos=dict(enabled=True,host='10.21.31.106'),gos=dict(enabled=True,host='10.21.31.104'))))
        c.known.write_text('10.21.31.106 ssh-ed25519 AAAA\n10.21.31.104 ssh-ed25519 AAAA\n')
        from unittest.mock import MagicMock
        nos=MagicMock();gos=MagicMock();nos.poll.return_value=None;gos.poll.return_value=None
        with patch('m20.connection_manager.subprocess.Popen',side_effect=[nos,gos]) as start:
            c.start('nos');c.start('gos')
            self.assertEqual(set(c.processes),{'nos','gos'})
            self.assertIn('127.0.0.1:8093:127.0.0.1:8093',start.call_args_list[0].args[0])
            self.assertIn('127.0.0.1:8090:127.0.0.1:8090',start.call_args_list[1].args[0])
            c.stop('gos');gos.terminate.assert_called_once();nos.terminate.assert_not_called()
            self.assertTrue(c.value['targets']['nos']['enabled'])
    def test_invalid_ros_data_source_rejected(self):
        with self.assertRaises(ValueError):self.connections.save(dict(sensor_source='aos'))
    def test_fingerprint_timeout_identifies_target_without_starting_keyscan(self):
        with patch('m20.connection_manager.socket.create_connection',side_effect=TimeoutError),patch('m20.connection_manager.subprocess.run') as keyscan:
            with self.assertRaisesRegex(ValueError,'10.21.31.104:22.*超时'):
                self.connections.scan('gos')
            keyscan.assert_not_called()
        self.assertFalse(self.connections.public()['targets']['gos']['trusted'])
    def test_fingerprint_reads_ssh_key_without_implicitly_trusting_host(self):
        stream=MagicMock();stream.__enter__.return_value=stream;stream.recv.return_value=b'SSH-2.0-OpenSSH_8.2\r\n'
        key=base64.b64encode(b'public-key-fixture').decode()
        output=SimpleNamespace(stdout='10.21.31.104 ssh-ed25519 '+key+'\n',returncode=0)
        with patch('m20.connection_manager.socket.create_connection',return_value=stream),patch('m20.connection_manager.subprocess.run',return_value=output):
            result=self.connections.scan('gos')
        self.assertEqual(result['host'],'10.21.31.104');self.assertEqual(result['port'],22)
        self.assertEqual(result['fingerprints'][0]['algorithm'],'ssh-ed25519')
        self.assertFalse(self.connections.public()['targets']['gos']['trusted'])
    def test_fingerprint_rejects_non_ssh_port(self):
        stream=MagicMock();stream.__enter__.return_value=stream;stream.recv.return_value=b'HTTP/1.1 200 OK\r\n'
        with patch('m20.connection_manager.socket.create_connection',return_value=stream),patch('m20.connection_manager.subprocess.run') as keyscan:
            with self.assertRaisesRegex(ValueError,'未返回 SSH 标识'):self.connections.scan('gos')
            keyscan.assert_not_called()
    def test_fingerprint_keyscan_has_bounded_timeout(self):
        stream=MagicMock();stream.__enter__.return_value=stream;stream.recv.return_value=b'SSH-2.0-OpenSSH_8.2\r\n'
        with patch('m20.connection_manager.socket.create_connection',return_value=stream),patch('m20.connection_manager.subprocess.run',side_effect=subprocess.TimeoutExpired('ssh-keyscan',10)):
            with self.assertRaisesRegex(ValueError,'公钥读取超时'):self.connections.scan('gos')
    def test_host_options_and_duplicate_forward_ports_rejected(self):
        for value in ['-oProxyCommand=evil','host;touch /tmp/x','user@host','a\nHost b']:
            with self.assertRaises(ValueError):host(value)
        with self.assertRaises(ValueError):
            self.connections.save(dict(targets=dict(nos=dict(enabled=True,host='host',local_port=9000),sdk=dict(enabled=True,host='host',local_port=9000))))
        with self.assertRaises(ValueError):target_config('nos',dict(domain_id=True))
    def test_ssh_requires_trust_and_never_disables_host_checking(self):
        c=self.connections;c.value['targets']['nos']['host']='robot'
        with self.assertRaises(ValueError):c.ssh_command('nos')
        c.known.write_text('robot ssh-ed25519 AAAA\n')
        argv,env=c.ssh_command('nos','ephemeral-password')
        self.assertIn('StrictHostKeyChecking=yes',argv)
        self.assertNotIn('ephemeral-password',' '.join(argv))
        self.assertEqual(env['M20_SSH_PASSWORD'],'ephemeral-password')
    def test_deployment_bundle_has_runtime_dependencies_without_local_data(self):
        with zipfile.ZipFile(io.BytesIO(base64.b64decode(Connections.bundle()))) as z:
            self.assertIn('bridge/mapping_job.py',z.namelist())
            self.assertIn('bridge/sdk_agent.py',z.namelist())
            self.assertFalse(any(name.startswith('data/') for name in z.namelist()))
            compile(z.read('bridge/install_remote.py') if 'bridge/install_remote.py' in z.namelist() else (ROOT/'bridge/install_remote.py').read_text(),'installer','exec')
    def test_installation_generates_working_service_entries_without_running_services(self):
        sandbox=Path(self.tmp.name)/'installation'
        account=SimpleNamespace(pw_uid=501,pw_gid=20)
        for role,onboard in [('nos',False),('nos',True),('gos',True),('sdk',False),('train',False)]:
            target=target_config(role,dict(host='robot',sdk_workspace='/home/user/M20_sdk_deploy',train_root='/home/user/rl_training',
                **({'resident_aos_ip' if role=='nos' else 'gos_aos_ip':'10.21.31.103'} if onboard else {})))
            payload=dict(bundle=Connections.bundle(),user='user',targets={role:target})
            source=ROOT_SETUP
            for location in ['/opt/m20-platform','/opt/robot','/var/lib/m20-platform','/etc/m20-platform','/etc/systemd/system','/etc/sudoers.d']:
                source=source.replace(location,str(sandbox/location.lstrip('/')))
                (sandbox/location.lstrip('/')).mkdir(parents=True,exist_ok=True)
            vendor=sandbox/'opt/robot/fastdds.xml'
            xml='<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><profiles><participant is_default_profile="true"><rtps><name>factory</name></rtps></participant></profiles></dds>'
            vendor.write_text(xml)
            with patch('pwd.getpwnam',return_value=account),patch('os.chown'),patch('subprocess.run',return_value=SimpleNamespace(returncode=0)),patch('sys.stdin',io.StringIO(json.dumps(payload))),patch('sys.stdout',io.StringIO()):
                exec(compile(source,'isolated-installer','exec'),{})
            unit=(sandbox/'etc/systemd/system'/('m20-platform-'+role+'.service')).read_text()
            entry='training_agent.py' if role=='train' else role+'_agent.py'
            self.assertIn(entry,unit)
            self.assertTrue((sandbox/'opt/m20-platform/bridge'/entry).is_file())
            self.assertIn('User=user',unit)
            self.assertNotIn('rl_deploy',unit)
            env=(sandbox/f'etc/m20-platform/{role}.env').read_text()
            if role=='nos' and onboard:
                self.assertIn('M20_RESIDENT_NAVIGATION=1\nM20_RESIDENT_AOS_IP=10.21.31.103\n',env)
            else:self.assertNotIn('M20_RESIDENT_NAVIGATION',env)
            if role=='gos':self.assertIn('M20_GOS_NAVIGATION=1\nM20_GOS_AOS_IP=10.21.31.103\n',env)
            if role in ['nos','gos']:
                import xml.etree.ElementTree as ET
                root=ET.parse(sandbox/('etc/m20-platform/sensors-'+role+'.xml'))
                self.assertEqual(next(x for x in root.iter() if x.tag.endswith('participantID')).text,'30' if role=='nos' else '31')
                self.assertEqual(vendor.read_text(),xml)
                reader=(sandbox/('etc/systemd/system/m20-platform-sensors-'+role+'.service')).read_text()
                self.assertIn('sensors-'+role+'.xml',reader);self.assertIn('UMask=0077',reader)


class HubTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.app=web.Platform(Store(Path(self.tmp.name)/'workspace.json'))
    def tearDown(self):self.app.hub.close();self.tmp.cleanup()
    def test_map_export_import_preview_sync_roundtrip(self):
        app=self.app;hub=app.hub
        exported=await_job(app,hub.post('/api/maps/package/export',dict(name='demo-yard')))
        meta=exported['file'];self.assertEqual(exported['format'],'directory-backup')
        await_job(app,hub.post('/api/maps/package/import',dict(id=meta['id'],name='imported-yard')))
        self.assertIn('imported-yard',[m['name'] for m in hub.get('/api/maps/library',{})['maps']])
        synced=hub.post('/api/maps/library/sync',dict(name='demo-yard',grid_id=0))
        self.assertEqual(synced['map']['grid_id'],0);self.assertTrue(synced['map']['verified_for_robot'])
        self.assertTrue((Path(self.tmp.name)/'maps'/(synced['map']['id']+'.png')).exists())
        cloud=await_job(app,hub.post('/api/cloud/preview',dict(kind='file',key=meta['id'])))
        self.assertGreater(cloud['shown_points'],100)
    def test_sync_reuses_map_id_and_preserves_routes_when_duplicate_caches_exist(self):
        hub=self.app.hub
        first=hub.sync_map(dict(name='demo-yard',grid_id=0))['map']
        duplicate=dict(first,id='aabbccddee')
        self.app.store.value['maps'].append(duplicate)
        route=self.app.route_save(dict(name='keep route',map_id=duplicate['id'],points=[dict(x=0,y=0)]))['route']
        second=hub.sync_map(dict(name='demo-yard',grid_id=0))['map']
        self.assertEqual(second['id'],first['id'])
        self.assertEqual(len([m for m in self.app.maps() if m.get('robot_map_name')=='demo-yard']),1)
        self.assertEqual(next(r for r in self.app.store.value['routes'] if r['id']==route['id'])['map_id'],first['id'])
    def test_navigation_sync_reuses_exact_grid_across_robot_addresses(self):
        hub=self.app.hub
        grid=hub.remote_maps('/maps/grid',dict(name='demo-yard',grid_id=0))
        source=hub.sync_map(dict(name='demo-yard',grid_id=0))['map']
        source.update(demo=False,robot_ip='192.0.2.1')
        route=self.app.route_save(dict(name='keep route',map_id=source['id'],points=[dict(x=0,y=0)]))['route']
        self.app.demo=False
        self.app.link=SimpleNamespace(ip='192.0.2.2')
        request=dict(name='demo-yard',grid_id=0,source_map_id=source['id'])
        with patch.object(hub,'remote_maps',return_value=grid) as remote:
            synced=hub.sync_map(request)
        self.assertNotIn('source_map_id',remote.call_args.args[1])
        self.assertEqual(synced['matched_source_map_id'],source['id'])
        self.assertEqual(synced['map']['id'],route['map_id'])
        self.assertEqual(synced['map']['robot_ip'],'192.0.2.2')
        raw=base64.b64decode(grid['pgm'])
        changed=dict(grid,pgm=base64.b64encode(raw[:-1]+bytes([raw[-1]^1])).decode())
        with patch.object(hub,'remote_maps',return_value=changed):
            with self.assertRaisesRegex(ValueError,'重新选点'):hub.sync_map(request)
        cached=Path(self.tmp.name)/'maps'/(source['id']+'.pgm')
        self.assertEqual(cached.read_bytes(),raw)
    def test_apply_also_synchronizes_navigation_and_reuses_map_id(self):
        hub=self.app.hub
        first=await_job(self.app,hub.post('/api/maps/library/apply',dict(name='demo-yard',grid_id=0)))
        self.assertTrue(first['navigation_ready']);self.assertTrue(first['map']['verified_for_robot'])
        second=await_job(self.app,hub.post('/api/maps/library/apply',dict(name='demo-yard',grid_id=0)))
        self.assertEqual(first['map']['id'],second['map']['id'])
    def test_synced_demo_map_is_labeled_and_hidden_from_live_map_selection(self):
        map_=self.app.hub.sync_map(dict(name='demo-yard',grid_id=0))['map']
        self.assertTrue(map_['demo']);self.app.demo=False
        self.assertNotIn(map_['id'],[m['id'] for m in self.app.maps()])
    def test_mapping_phase_does_not_confuse_launcher_exit_with_finished_mapping(self):
        start=dict(state='command_finished',started=100,name='room')
        result=mapping_job.mapping_phase(start,dict(state='none'),[])
        self.assertEqual(result['phase'],'unconfirmed');self.assertTrue(result['in_progress'])
        interrupted=mapping_job.mapping_phase(dict(start,processes_alive=[]),dict(state='none'),[])
        self.assertEqual(interrupted['phase'],'interrupted');self.assertFalse(interrupted['in_progress'])
        start['processes_alive']=[123]
        self.assertEqual(mapping_job.mapping_phase(start,dict(state='none'),[])['phase'],'collecting')
    def test_mapping_saved_requires_stop_completion_and_new_complete_map(self):
        start=dict(state='command_finished',started=100,name='room');stop=dict(state='command_finished',started=130,ended=160)
        self.assertEqual(mapping_job.mapping_phase(start,stop,[])['phase'],'saving')
        maps=[dict(name='room-123',modified=150,has_cloud=True,grids=[0])]
        result=mapping_job.mapping_phase(start,stop,maps)
        self.assertEqual(result['phase'],'saved');self.assertFalse(result['in_progress']);self.assertEqual(result['result_maps'],['room-123'])
        maps[0]['modified']=90
        self.assertTrue(mapping_job.mapping_phase(start,stop,maps)['in_progress'])
    def test_mapping_requires_standing_and_no_sdk_output(self):
        self.app.basic['MotionState']=4
        with self.assertRaises(ValueError):self.app.hub.post('/api/maps/library/build',dict(action='start',name='survey'))
        self.app.hub.sdk_mode=dict(state='sdk')
        with self.assertRaises(ValueError):self.app.hub.post('/api/maps/library/apply',dict(name='demo-yard'))
    def test_policy_stop_is_available_during_sdk_velocity(self):
        app=self.app;hub=app.hub
        hub.sdk_mode=dict(state='sdk',source='demo')
        hub.post('/api/sdk/policy/start',{})
        hub.post('/api/sdk/policy/state',dict(state='rl_control'))
        hub.post('/api/sdk/policy/velocity',dict(forward=.1))
        hub.post('/api/sdk/policy/stop',{})
        self.assertFalse(hub.demo_policy['vendor_running']);self.assertEqual(app.velocity,[0.,0.,0.])
    def test_policy_and_direct_joint_control_are_exclusive_in_demo(self):
        self.app.hub.demo_policy['vendor_running']=True
        with self.assertRaises(ValueError):self.app.joint_control('arm',dict(rig_confirmed=True))
        self.assertFalse(self.app.sdk['armed'])
    def test_global_stop_reaches_sdk_policy_without_joint_session(self):
        app=self.app;app.demo=False;app.config['sdk']='http://127.0.0.1:8091'
        app.hub.sdk_mode=dict(state='sdk',source='operator_confirmed')
        with patch.object(app,'proxy',return_value={'ok':True}) as call:
            app.stop()
            self.assertEqual(call.call_args.args[:2],('sdk','/stop'))
    def test_sdk_supervisor_startup_is_passive(self):
        agent=SDKAgent(workspace=Path(self.tmp.name)/'sdk',data=Path(self.tmp.name)/'agent')
        with patch('sdk_agent.RosPolicy',side_effect=AssertionError('Must not initialize control on connect')):
            status=agent.dispatch('/status',None)
        self.assertFalse(status['armed']);self.assertFalse(status['vendor_running']);self.assertIsNone(agent.worker)
        with self.assertRaises(ValueError):agent.dispatch('/policy/start',{})
    def test_gos_status_is_passive_and_never_starts_ros_or_motion(self):
        agent=GOSAgent(Path(self.tmp.name)/'gos')
        with patch('bridge.gos_agent.subprocess.run',side_effect=AssertionError('Status must not execute commands')):
            status=agent.dispatch('/status',None)
        self.assertEqual(status['role'],'gos');self.assertIsNone(agent.sensors)
        with self.assertRaises(ValueError):agent.dispatch('/arm',dict())
    def test_gos_ros_source_does_not_move_nos_map_operations(self):
        app=self.app;hub=app.hub;app.demo=False
        hub.connections.save(dict(sensor_source='gos'))
        with patch.object(hub,'proxy',return_value=dict(ok=True,topics=[])) as proxy:
            result=hub.get('/api/sensors/status',{})
            self.assertEqual(result['host_role'],'gos');self.assertEqual(proxy.call_args.args[0],'gos')
            hub.get('/api/maps/library',{})
            self.assertEqual(proxy.call_args.args[0],'nos')
    def test_sdk_source_prepare_refuses_to_overwrite_existing_source(self):
        workspace=Path(self.tmp.name)/'sdk';source=workspace/'src/M20_sdk_deploy';source.mkdir(parents=True)
        (source/'main.cpp').write_text('existing source')
        agent=SDKAgent(workspace=workspace,data=Path(self.tmp.name)/'agent')
        with patch('sdk_agent.subprocess.run') as run:
            with self.assertRaises(ValueError):agent.prepare_source(dict(confirmed=True))
            run.assert_not_called()
        self.assertEqual((source/'main.cpp').read_text(),'existing source')
    def test_sdk_velocity_watchdog_sends_zero_without_browser(self):
        policy=RosPolicy.__new__(RosPolicy);published=[]
        policy.lock=threading.RLock();policy.rx=time.monotonic();policy.velocity=[0.,0.,0.];policy.until=0;policy.pending_zero=False
        policy.state=dict(current_state_id=6,source='ros2',telemetry_fresh=True,safety_mode=0)
        policy.Twist=lambda:SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.),angular=SimpleNamespace(z=0.))
        policy.velocities=SimpleNamespace(publish=lambda msg:published.append([msg.linear.x,msg.linear.y,msg.angular.z]))
        policy.set_velocity(dict(forward=.1));policy.tick();self.assertEqual(published[-1],[.1,0.,0.])
        policy.until=0;policy.tick();self.assertEqual(published[-1],[0.,0.,0.])
        policy.state['safety_mode']=1
        with self.assertRaises(ValueError):policy.set_velocity(dict(forward=.1))
    def test_mapping_journal_distinguishes_command_exit_from_business_result(self):
        logs=Path(self.tmp.name)/'mapping';logs.mkdir()
        def run(argv,**options):
            self.assertEqual(argv,['drmap','mapping','-s','-b','-n','survey'])
            options['stdout'].write('Building map\ncommand exited\n')
            return SimpleNamespace(returncode=0)
        with patch('bridge.mapping_job.subprocess.run',side_effect=run):mapping_job.worker('mapping',logs,'survey')
        result=mapping_job.read_job('mapping',logs)
        self.assertEqual(result['state'],'command_finished');self.assertIn('Building map',result['output'])
        self.assertIn('核对',result['message'])
    def test_duplicate_mapping_launch_is_reserved_before_child_starts(self):
        logs=Path(self.tmp.name)/'mapping'
        with patch('bridge.mapping_job.subprocess.Popen',return_value=SimpleNamespace(pid=42)) as start:
            mapping_job.launch('mapping','survey',logs)
            with self.assertRaises(ValueError):mapping_job.launch('mapping','survey',logs)
            self.assertEqual(start.call_count,1)
    def test_busy_blocks_motion_and_releases_on_failure(self):
        hub=self.app.hub;gate=threading.Event()
        job=hub.mutate('test-operation',lambda progress:gate.wait(2))
        with self.assertRaises(ValueError):self.app.set_velocity(dict(forward=.1))
        self.app.stop();gate.set();await_job(self.app,job)
        self.assertIsNone(hub.busy)
    def test_lift_preserves_body_xy_with_abducted_leg(self):
        for leg in ['FL','FR','HL','HR']:
            q=list(NEUTRAL);index=['FL','FR','HL','HR'].index(leg)*4;q[index]=.2
            before=leg_forward(q,leg);target=leg_target(q,leg,.04);after=leg_forward(target,leg)
            self.assertAlmostEqual(after[0],before[0],places=8)
            self.assertAlmostEqual(after[1],before[1],places=8)
            self.assertAlmostEqual(after[2]-before[2],.04,places=8)
            self.assertEqual(target[index+3],q[index+3])
            self.assertEqual(target[:index],q[:index]);self.assertEqual(target[index+4:],q[index+4:])
        preview=self.app.joint_control('preview',dict(leg='FL',height=.04))
        self.assertIn('positions',preview);self.assertFalse(self.app.sdk['armed'])


class WorkflowHTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();web.APP=web.Platform(Store(Path(self.tmp.name)/'workspace.json'));web.TOKEN='workflow-test'
        self.server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler);self.url='http://127.0.0.1:'+str(self.server.server_address[1])
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();web.APP.hub.close();self.tmp.cleanup();web.TOKEN=None
    def call(self,path,body=None):
        headers={'X-API-Token':'workflow-test','Content-Type':'application/json'}
        with urlopen(Request(self.url+path,data=json.dumps(body).encode() if body is not None else None,headers=headers),timeout=3) as r:return json.load(r)
    def test_v2_endpoints_and_no_typesafe_dependency(self):
        self.assertIn('targets',self.call('/api/connections')['connections'])
        self.assertIn('maps',self.call('/api/maps/library'))
        self.assertTrue(self.call('/api/sdk/status')['demo'])
        self.assertNotIn('ai_available',self.call('/api/platform'))
        response=self.call('/api/cloud/preview',dict(kind='map',key='demo-yard'))
        cloud=await_job(web.APP,response)
        self.assertGreater(self.call('/api/cloud/data/'+cloud['cloud_id'])['shown_points'],100)
        with self.assertRaises(HTTPError):self.call('/api/assistant',dict(text='SDK'))
    def test_physical_host_configuration_and_demo_gos_status(self):
        public=self.call('/api/connections')['connections']
        self.assertIn('aos_ip',public);self.assertIn('nos',public['targets']);self.assertIn('gos',public['targets'])
        self.assertTrue(self.call('/api/gos/status')['demo'])
        self.assertFalse(public['targets']['gos']['enabled'])


if __name__=='__main__':unittest.main()
