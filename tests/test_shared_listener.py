"""Network-change startup fallback; only mocked interfaces and servers."""
import sys
import tempfile
import unittest
from unittest.mock import patch
from m20.shared_access import shared_listener


class SharedListenerTests(unittest.TestCase):
    def test_missing_configured_wifi_keeps_localhost_available(self):
        with patch('m20.shared_access.lan_addresses',return_value=[]):
            access,bind,message=shared_listener('unused','192.168.88.0/24',8080)
        self.assertIsNone(access);self.assertEqual(bind,'127.0.0.1');self.assertIn('配置保留',message)

    def test_matching_wifi_keeps_the_existing_private_scope(self):
        with patch('m20.shared_access.lan_addresses',return_value=['192.168.88.8']):
            access,bind,message=shared_listener('unused','192.168.88.0/24',8080)
        self.assertEqual(bind,'0.0.0.0');self.assertIsNone(message)
        self.assertEqual(access.urls,['http://192.168.88.8:8080/'])
        self.assertFalse(access.admitted('192.168.99.8'))

    def test_invalid_or_public_scope_is_not_silently_accepted(self):
        for value in ['bad-network','0.0.0.0/0','8.8.8.0/24']:
            with self.assertRaises(ValueError):shared_listener('unused',value,8080)


class SharedStartupTests(unittest.TestCase):
    def test_actual_demo_startup_uses_local_listener_when_wifi_is_absent(self):
        import m20_platform as web
        addresses=[]
        class Server:
            def __init__(self,address,handler):addresses.append(address)
            def serve_forever(self):raise KeyboardInterrupt()
            def server_close(self):pass
        class Thread:
            def __init__(self,*args,**kwargs):pass
            def start(self):pass
        old=(web.APP,web.ACCESS,web.TOKEN)
        try:
            with tempfile.TemporaryDirectory() as folder,patch.object(sys,'argv',['m20_platform.py','--demo','--data-dir',folder,'--share','192.168.88.0/24']),patch('m20.shared_access.lan_addresses',return_value=[]),patch.object(web,'ThreadingHTTPServer',Server),patch.object(web.threading,'Thread',Thread):
                web.main()
                self.assertEqual(addresses,[('127.0.0.1',8080)])
                self.assertTrue(web.APP.demo);self.assertIsNone(web.APP.link);self.assertIsNone(web.ACCESS)
        finally:web.APP,web.ACCESS,web.TOKEN=old


if __name__=='__main__':unittest.main()
