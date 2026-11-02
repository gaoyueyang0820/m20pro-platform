"""Private Wi-Fi access is passwordless; tests use only a local demo server."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from shared_access import SharedAccess
import m20_platform as web
from platform_core import Store

class SharedAccessTests(unittest.TestCase):
    def test_private_wifi_needs_no_password_or_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=SharedAccess(tmp,'172.20.10.0/28')
            self.assertTrue(a.admitted('172.20.10.8'));self.assertTrue(a.admitted('127.0.0.1'))
            self.assertFalse(a.admitted('172.20.10.18'));self.assertFalse(a.admitted('100.64.1.30'))
            self.assertFalse(a.status(peer='172.20.10.8')['authentication_required'])
            self.assertTrue(a.status(peer='172.20.10.8')['controlling'])
            self.assertEqual(list(Path(tmp).iterdir()),[])
            with self.assertRaises(ValueError):SharedAccess(tmp,'0.0.0.0/0')

class SharedHTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();web.APP=web.Platform(Store(Path(self.tmp.name)/'workspace.json'))
        web.TOKEN=None;web.ACCESS=SharedAccess(self.tmp.name,'172.20.10.0/28')
        self.server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
        self.url='http://127.0.0.1:'+str(self.server.server_address[1]);web.ACCESS.urls=['http://172.20.10.4:'+str(self.server.server_address[1])+'/']
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
        self.client=build_opener(ProxyHandler({}))
    def tearDown(self):
        self.server.shutdown();self.server.server_close();web.APP.hub.close();web.ACCESS=None;self.tmp.cleanup()
    def call(self,path,body=None,headers=None):
        req=Request(self.url+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json',**(headers or {})})
        with self.client.open(req,timeout=3) as response:return json.load(response)
    def test_no_login_or_control_claim_required_on_local_or_wifi(self):
        self.assertFalse(self.call('/api/auth')['required'])
        original=web.Handler._admitted
        def remote(handler):
            handler.client_address=('172.20.10.8',handler.client_address[1]);return original(handler)
        with patch.object(web.Handler,'_admitted',remote),patch.object(web.APP,'command',return_value={'ok':True}) as command:
            self.assertEqual(self.call('/api/platform')['mode'],'demo')
            self.call('/api/cmd',{'action':'stand'});command.assert_called_once_with('stand')
            status=self.call('/api/sharing');self.assertTrue(status['controlling']);self.assertFalse(status['authentication_required'])
        with self.assertRaises(HTTPError) as err:self.call('/api/sharing/password',{})
        self.assertEqual(err.exception.code,410)
    def test_outside_selected_wifi_stays_outside_shared_scope(self):
        original=web.Handler._admitted
        def remote(handler):
            handler.client_address=('192.168.5.8',handler.client_address[1]);return original(handler)
        with patch.object(web.Handler,'_admitted',remote):
            with self.assertRaises(HTTPError) as err:self.call('/api/platform')
            self.assertEqual(err.exception.code,403)
    def test_no_cookie_required_after_server_restarts(self):
        self.call('/api/platform');web.ACCESS=SharedAccess(self.tmp.name,'172.20.10.0/28')
        self.assertTrue(self.call('/api/sharing')['controlling'])
        self.assertEqual(self.call('/api/platform')['mode'],'demo')

if __name__=='__main__':unittest.main()
