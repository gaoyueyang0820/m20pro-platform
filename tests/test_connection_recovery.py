"""Disconnected desktop reconnection, using synthetic links and no network."""
import time
import unittest
from unittest.mock import Mock,patch
import test_platform as cases


class DeferredThread:
    def __init__(self,target,**kwargs):self.target=target;self.running=False
    def start(self):self.running=True
    def is_alive(self):return self.running
    def run(self):
        try:self.target()
        finally:self.running=False


class ConnectionRecoveryTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def offline(self):
        self.app.demo=False;self.app.link=cases.FakeLink();self.app.link.ip='192.0.2.10'
        self.app.link.records[1002,6]['at']=time.time()-30
        self.app.hub._connection_lost_since=time.monotonic()-6
        self.app.hub.connections.value['aos_ip']='192.0.2.20'
        self.app.connect=Mock(return_value={'ok':True})
    def test_fresh_connection_never_starts_network_discovery(self):
        self.offline();self.app.link.records[1002,6]['at']=time.time()
        with patch('m20.platform_hub.threading.Thread') as thread:self.app.hub.maintain_connection()
        thread.assert_not_called();self.app.connect.assert_not_called()
    def test_background_search_restores_new_address_once_without_resuming_jobs(self):
        self.offline();hub=self.app.hub
        with patch('m20.platform_hub.threading.Thread',DeferredThread),patch.object(hub.connections,'discover',return_value={'found':True}) as discover:
            hub.maintain_connection();worker=hub._connection_search;hub.maintain_connection();worker.run();hub.maintain_connection()
        discover.assert_called_once();self.app.connect.assert_called_once_with({'mode':'live','ip':'192.0.2.20','port':30000})
    def test_newer_connection_or_recovered_current_link_is_not_replaced(self):
        for change in ['context','fresh']:
            self.offline();hub=self.app.hub;hub._connection_search_at=-30
            with patch('m20.platform_hub.threading.Thread',DeferredThread),patch.object(hub.connections,'discover',return_value={'found':True}):
                hub.maintain_connection()
                if change=='context':hub._context_generation+=1
                else:self.app.link.records[1002,6]['at']=time.time()
                hub._connection_search.run()
            self.app.connect.assert_not_called()
    def test_search_result_never_switches_demonstration_to_live(self):
        self.offline();hub=self.app.hub
        with patch('m20.platform_hub.threading.Thread',DeferredThread),patch.object(hub.connections,'discover',return_value={'found':True}):
            hub.maintain_connection();self.app.demo=True;hub._connection_search.run()
        self.app.connect.assert_not_called()


if __name__=='__main__':unittest.main()
