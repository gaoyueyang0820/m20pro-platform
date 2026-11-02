"""Synthetic adapter inventories and public keys; no network or host commands.

10.253.254.0/24 and neighboring ranges below are invented LAN fixtures. Saved
endpoints use RFC 5737 documentation ranges, never the developer's robot IPs.
"""
import base64
import copy
import ipaddress
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import MagicMock,patch

from connection_manager import Connections,target_config
import robot_discovery as discovery
from ssh_identity import write_alias


def public_key(seed=1):
    algorithm=b'ssh-ed25519'
    return base64.b64encode(struct.pack('>I',len(algorithm))+algorithm+struct.pack('>I',32)+bytes([seed])*32).decode()


def interface(address='10.253.254.42/24',name='wifi-fixture'):
    return dict(name=name,address=ipaddress.IPv4Interface(address))


class CandidateTests(unittest.TestCase):
    def test_saved_and_attached_neighbors_are_first_own_and_vpn_neighbors_excluded(self):
        rows=[interface()]
        neighbors=[dict(name='wifi-fixture',address='10.253.254.99'),dict(name='utun9',address='10.253.254.98'),dict(name='wifi-fixture',address='198.51.100.4')]
        values=discovery.candidate_addresses('192.0.2.10',rows,neighbors)
        self.assertEqual(values[:2],['192.0.2.10','10.253.254.99'])
        self.assertNotIn('10.253.254.42',values);self.assertNotIn('198.51.100.4',values)
        self.assertEqual(len(values),254);self.assertEqual(len(set(values)),len(values))
        self.assertNotIn('10.253.254.0',values);self.assertNotIn('10.253.254.255',values)

    def test_large_subnet_scans_current_24_only_and_can_check_known_neighbors_elsewhere(self):
        values=discovery.candidate_addresses('192.0.2.10',[interface('10.253.254.42/16')],[dict(name='wifi-fixture',address='10.253.240.99')])
        self.assertEqual(values[:2],['192.0.2.10','10.253.240.99'])
        self.assertTrue(all(value.startswith('10.253.254.') for value in values[2:]));self.assertLessEqual(len(values),256)

    def test_multiple_physical_adapters_share_candidate_budget(self):
        values=discovery.candidate_addresses(None,[interface(),interface('10.254.253.42/16','ethernet-fixture')],[],limit=10)
        self.assertEqual(len(values),10);self.assertEqual(values[:4],['10.253.254.1','10.254.253.1','10.253.254.2','10.254.253.2'])
        values=discovery.candidate_addresses(None,[interface('10.253.1.1/8'),interface('10.254.2.2/8'),interface('10.252.3.3/8')],[])
        self.assertEqual(len(values),discovery.MAX_CANDIDATES)

    def test_public_and_linklocal_interfaces_are_not_swept(self):
        rows=[interface('198.51.100.2/24'),interface('169.254.100.2/16','ethernet-fixture')]
        self.assertEqual(discovery.candidate_addresses('192.0.2.10',rows,[]),['192.0.2.10'])
        self.assertEqual(discovery.candidate_addresses('not-an-ip',[],[]),[])


class InventoryTests(unittest.TestCase):
    def test_macos_uses_hardware_ports_not_vpn_default_route(self):
        outputs={
            'networksetup':'Hardware Port: Wi-Fi\nDevice: en0\nHardware Port: Ethernet\nDevice: en7\nHardware Port: Thunderbolt Bridge\nDevice: bridge0\n',
            'ifconfig':'en0: flags=8863<UP,BROADCAST,RUNNING> mtu 1500\n\tinet 10.253.254.42 netmask 0xffffff00 broadcast 10.253.254.255\n\tstatus: active\nen7: flags=8863<UP,BROADCAST,RUNNING> mtu 1500\n\tinet 10.254.253.42 netmask 0xffffff00\n\tstatus: inactive\nutun9: flags=8051<UP,POINTOPOINT,RUNNING> mtu 1500\n\tinet 10.252.1.1 netmask 0xffffff00\nbridge0: flags=8863<UP,BROADCAST,RUNNING> mtu 1500\n\tinet 10.251.1.1 netmask 0xffffff00\n',
            'arp':'? (10.253.254.99) at 00:11:22:33:44:55 on en0 ifscope [ethernet]\n? (10.252.1.2) at 00:11:22:33:44:55 on utun9\n? (10.253.254.98) at (incomplete) on en0\n'}
        with patch.object(discovery,'_capture',side_effect=lambda argv:outputs[argv[0]]) as capture:
            interfaces,neighbors=discovery.network_inventory('darwin')
        self.assertEqual(interfaces,[interface(name='en0')]);self.assertEqual(neighbors,[dict(name='en0',address='10.253.254.99')])
        self.assertNotIn('route',[call.args[0][0] for call in capture.call_args_list])

    def test_linux_ignores_virtual_link_and_neighbors_without_physical_interfaces(self):
        rows={
            ('ip','-j','link','show'):[dict(ifname='wifi0',link_type='ether',flags=['UP']),dict(ifname='tun0',link_type='none',flags=['UP']),dict(ifname='eth1',link_type='ether',flags=['UP'],linkinfo=dict(info_kind='veth')),dict(ifname='fake0',link_type='ether',flags=['UP'])],
            ('ip','-j','-4','address','show'):[dict(ifname='wifi0',addr_info=[dict(family='inet',local='10.253.254.42',prefixlen=24)]),dict(ifname='tun0',addr_info=[dict(family='inet',local='10.252.1.1',prefixlen=24)])],
            ('ip','-j','-4','neigh','show'):[dict(dev='wifi0',dst='10.253.254.99',state=['STALE']),dict(dev='wifi0',dst='10.253.254.98',state=['FAILED']),dict(dev='tun0',dst='10.252.1.2',state=['REACHABLE'])]}
        with patch.object(discovery,'_capture',side_effect=lambda argv:json.dumps(rows[tuple(argv)])),patch.object(discovery,'_linux_physical',side_effect=lambda name:name=='wifi0'):
            interfaces,neighbors=discovery.network_inventory('linux')
        self.assertEqual(interfaces,[interface(name='wifi0')]);self.assertEqual(neighbors,[dict(name='wifi0',address='10.253.254.99')])

    def test_windows_uses_physical_adapter_indices_and_attached_neighbors(self):
        inventory=dict(adapters=[dict(Name='Wi-Fi',InterfaceIndex=7)],addresses=[dict(InterfaceIndex=7,IPAddress='10.253.254.42',PrefixLength=24),dict(InterfaceIndex=9,IPAddress='10.252.1.1',PrefixLength=24)],
                       neighbors=[dict(InterfaceIndex=7,IPAddress='10.253.254.99',State='Reachable'),dict(InterfaceIndex=9,IPAddress='10.252.1.2',State='Stale'),dict(InterfaceIndex=7,IPAddress='10.253.254.98',State='Incomplete')])
        with patch.object(discovery,'_capture',return_value=json.dumps(inventory)) as capture:
            interfaces,neighbors=discovery.network_inventory('win32')
        self.assertIn('Get-NetAdapter -Physical',capture.call_args.args[0][-1]);self.assertEqual(interfaces,[interface(name='7')]);self.assertEqual(neighbors,[dict(name='7',address='10.253.254.99')])

    def test_missing_inventory_tools_return_empty_without_other_network_fallbacks(self):
        with patch.object(discovery.subprocess,'run',side_effect=FileNotFoundError('fixture')) as run:
            self.assertEqual(discovery.network_inventory('darwin'),([],[]));self.assertEqual(discovery.network_inventory('linux'),([],[]));self.assertEqual(discovery.network_inventory('win32'),([],[]))
        self.assertFalse(any('route' in str(call) for call in run.call_args_list))


class IdentitySearchTests(unittest.TestCase):
    def test_open_ssh_endpoint_requires_complete_paired_key_and_bounded_tools(self):
        expected={('ssh-ed25519',public_key())};stop=threading.Event()
        for key,wanted in [(public_key(9),False),(public_key(),True)]:
            with self.subTest(key=key),patch.object(discovery.socket,'create_connection',return_value=MagicMock()) as connect,patch.object(discovery.platform_host,'ssh_executable',side_effect=lambda name:name),\
                 patch.object(discovery.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'198.51.100.8 ssh-ed25519 '+key+'\n','')) as scan:
                self.assertEqual(discovery._matches('198.51.100.8',2222,expected,stop),wanted)
                self.assertEqual(connect.call_args.kwargs['timeout'],.22);self.assertEqual(scan.call_args.kwargs['timeout'],2)
                self.assertIn('2222',scan.call_args.args[0]);self.assertEqual(scan.call_args.args[0][0],'ssh-keyscan')
        with patch.object(discovery.socket,'create_connection',side_effect=TimeoutError('fixture')),patch.object(discovery.subprocess,'run') as scan:
            self.assertFalse(discovery._matches('198.51.100.8',22,expected,stop));scan.assert_not_called()

    def test_unknown_robot_never_enumerates_or_scans(self):
        with patch.object(discovery,'network_inventory') as inventory,patch.object(discovery,'_matches') as matches:
            self.assertIsNone(discovery.find_robot('192.0.2.10',22,[]));inventory.assert_not_called();matches.assert_not_called()

    def test_saved_matching_address_avoids_subnet_scan(self):
        with patch.object(discovery,'network_inventory') as inventory,patch.object(discovery,'_matches',return_value=True) as matches:
            self.assertEqual(discovery.find_robot('192.0.2.10',22,[('ssh-ed25519',public_key())]),'192.0.2.10')
            inventory.assert_not_called();self.assertEqual(matches.call_count,1)

    def test_changed_network_checks_candidates_concurrently_and_returns_only_match(self):
        barrier=threading.Barrier(2);workers=set()
        def match(address,port,expected,stop):
            if address=='192.0.2.10':return False
            workers.add(threading.get_ident());barrier.wait(1)
            return address=='198.51.100.8'
        with patch.object(discovery,'network_inventory',return_value=([interface()],[])),\
             patch.object(discovery,'candidate_addresses',side_effect=lambda saved,rows,neighbors:['192.0.2.10','198.51.100.7','198.51.100.8'] if rows else ['192.0.2.10']),\
             patch.object(discovery,'_matches',side_effect=match):
            self.assertEqual(discovery.find_robot('192.0.2.10',22,[('ssh-ed25519',public_key())]),'198.51.100.8')
        self.assertEqual(len(workers),2)

    def test_cancellation_discards_even_a_completed_matching_scan(self):
        cancel=threading.Event()
        def match(*args):cancel.set();return True
        with patch.object(discovery,'_matches',side_effect=match),patch.object(discovery,'network_inventory') as inventory:
            self.assertIsNone(discovery.find_robot('192.0.2.10',22,[('ssh-ed25519',public_key())],cancel=cancel.is_set));inventory.assert_not_called()


class ConnectionsDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.files=patch('platform_host.private_file',lambda path:None);self.files.start()
        self.c=Connections(Path(self.temp.name));self.alias='m20-aos-'+'a'*32
        self.c.value.update(aos_ip='192.0.2.10',aos_identity_alias=self.alias,auto_connect=True)
        self.c.value['targets']['sdk']=target_config('sdk',dict(host='192.0.2.10',follows_aos=True,enabled=True),{})
        self.c.value['targets']['sdk']['deployed']=True
        for role in ['nos','gos']:
            self.c.value['targets'][role]=target_config(role,dict(host=role+'.fixture',via_aos=True,enabled=True),{})
            self.c.value['targets'][role]['deployed']=True
        write_alias(self.c.known,self.alias,[('ssh-ed25519',public_key())]);self.c.persist()
    def tearDown(self):self.c.close();self.files.stop();self.temp.cleanup()

    def test_matching_discovery_moves_aos_and_following_sdk_preserving_service_and_auto_connect(self):
        c=self.c
        with patch.object(discovery,'find_robot',return_value='198.51.100.8') as find:
            result=c.discover()
        self.assertTrue(result['found']);self.assertEqual(c.value['aos_ip'],'198.51.100.8');self.assertEqual(c.value['targets']['sdk']['host'],'198.51.100.8')
        self.assertTrue(c.value['auto_connect']);self.assertTrue(c.value['targets']['sdk']['deployed']);self.assertEqual(c.value['aos_identity_alias'],self.alias)
        self.assertTrue(all(c.value['targets'][role]['deployed'] for role in ['nos','gos']))
        self.assertNotIn('aos_identity_alias',result['connections']);self.assertEqual(find.call_args.args[:3],('192.0.2.10',22,[('ssh-ed25519',public_key())]))

    def test_no_match_keeps_configuration_and_known_keys(self):
        before=copy.deepcopy(self.c.value);known=self.c.known.read_text()
        with patch.object(discovery,'find_robot',return_value=None):result=self.c.discover()
        self.assertFalse(result['found']);self.assertEqual(self.c.value,before);self.assertEqual(self.c.known.read_text(),known)

    def test_unpaired_profile_returns_manual_connection_hint_without_scan(self):
        self.c.value.pop('aos_identity_alias')
        with patch.object(discovery,'find_robot') as find:result=self.c.discover()
        self.assertFalse(result['found']);self.assertIn('首次连接',result['message']);find.assert_not_called()

    def test_user_address_change_during_scan_wins_over_discovery(self):
        def find(*args,**kwargs):self.c.save(dict(aos_ip='198.51.100.9',auto_connect=True));return '198.51.100.8'
        with patch.object(discovery,'find_robot',side_effect=find):result=self.c.discover()
        self.assertFalse(result['found']);self.assertEqual(self.c.value['aos_ip'],'198.51.100.9')

    def test_identity_rotation_during_scan_cannot_select_old_robot(self):
        def find(*args,**kwargs):write_alias(self.c.known,self.alias,[('ssh-ed25519',public_key(9))]);return '198.51.100.8'
        with patch.object(discovery,'find_robot',side_effect=find):result=self.c.discover()
        self.assertFalse(result['found']);self.assertEqual(self.c.value['aos_ip'],'192.0.2.10')

    def test_external_sdk_target_does_not_follow_discovered_aos(self):
        self.c.save(dict(targets=dict(sdk=dict(host='sdk.fixture',follows_aos=False)),auto_connect=True))
        with patch.object(discovery,'find_robot',return_value='198.51.100.8'):self.c.discover()
        self.assertEqual(self.c.value['targets']['sdk']['host'],'sdk.fixture')


if __name__=='__main__':unittest.main()
