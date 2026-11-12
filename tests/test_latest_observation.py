"""Deterministic cache semantics plus thread-generation handoff, no network."""
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import URLError,HTTPError
import io
from m20.observation_errors import BridgeTransportError
from m20.platform_core import rpc
from m20.latest_observation import LatestObservationPump,TransientObservationError


class PumpTests(unittest.TestCase):
    def setUp(self):self.pumps=[]
    def tearDown(self):
        for p in self.pumps:p.cancel();p.join(.3)

    def pump(self):
        clock=[100.]
        def fetch(context,stop):stop.wait(2);return dict(stamp=1.,_valid_until=0.)
        p=LatestObservationPump(fetch,clock=lambda:clock[0]);self.pumps.append(p);generation=p.start({'robot':'captured'})
        return p,generation,clock

    def packet(self,stamp=10.,deadline=100.3):
        return dict(stamp=stamp,_valid_until=deadline,age=.35,floor=[[5,0]],obstacles=[],pose=[0,0,0])

    def test_reads_do_not_wait_for_fetch_and_do_not_extend_deadline(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        first=p.read(g);c[0]=100.2;again=p.read(g,first['sequence'])
        self.assertEqual(again['state'],'cached');self.assertAlmostEqual(again['remaining_s'],.1)
        self.assertEqual(again['observation']['_valid_until'],100.3)
        self.assertAlmostEqual(again['observation']['age'],.55)
        hold=p.read(g,first['sequence'],fresh_only=True)
        self.assertEqual((hold['state'],hold['observation'],hold['valid_until']),('cached',None,100.3))
        c[0]=100.31;self.assertEqual(p.read(g)['state'],'expired')
        self.assertIsNone(p.read(g)['observation'])

    def test_duplicate_source_stamp_cannot_renew_or_replace_geometry(self):
        p,g,c=self.pump();p._publish(self.packet(),g);c[0]=100.2
        duplicate=self.packet(deadline=100.7);duplicate['floor']=[[99,99]]
        self.assertFalse(p._publish(duplicate,g))
        value=p.read(g);self.assertEqual(value['sequence'],1)
        self.assertEqual(value['observation']['floor'],[[5,0]])
        self.assertEqual(value['observation']['_valid_until'],100.3)
        c[0]=100.4;p._publish(self.packet(deadline=100.8),g)
        self.assertEqual(p.read(g)['state'],'expired')

    def test_duplicate_can_only_shorten_original_deadline(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        p._publish(self.packet(deadline=100.15),g)
        self.assertEqual(p.read(g)['observation']['_valid_until'],100.15)

    def test_out_of_order_and_expired_new_packet_cannot_replace_valid_cache(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        self.assertFalse(p._publish(self.packet(stamp=9.,deadline=100.5),g))
        self.assertFalse(p._publish(self.packet(stamp=11.,deadline=99.9),g))
        self.assertEqual(p.read(g)['observation']['stamp'],10.)
        c[0]=100.4;self.assertEqual(p.read(g)['state'],'expired')

    def test_missing_or_invented_deadline_invalidates_cache(self):
        for payload in [dict(stamp=11.),self.packet(stamp=float('nan')),self.packet(deadline=101.)]:
            p,g,c=self.pump();p._publish(self.packet(),g)
            self.assertFalse(p._publish(payload,g));self.assertEqual(p.read(g)['state'],'invalidated')

    def test_transport_error_preserves_only_remaining_lease_geometry_error_drops_it(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        p._error(TransientObservationError('timed out'),g)
        self.assertEqual(p.read(g)['state'],'fresh')
        c[0]=100.31;self.assertEqual(p.read(g)['state'],'expired')
        p._publish(self.packet(stamp=11.,deadline=100.6),g)
        p._error(ValueError('pose invalid'),g)
        self.assertEqual(p.read(g)['state'],'invalidated')

    def test_real_rpc_network_failure_keeps_valid_cache_until_original_expiry(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        with patch('m20.platform_core.urlopen',side_effect=URLError('synthetic timeout')):
            for now in [100.1,100.2,100.29]:
                c[0]=now
                with self.assertRaises(BridgeTransportError) as error:rpc('http://offline.invalid','/sensors/autonomy',{'mapping':True})
                p._error(error.exception,g)
                self.assertEqual(p.read(g)['observation']['_valid_until'],100.3)
        c[0]=100.31;self.assertEqual(p.read(g)['state'],'expired')

    def test_real_rpc_geometry_failure_still_drops_cached_scene_immediately(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        response=HTTPError('http://offline.invalid',400,'synthetic',{},io.BytesIO(b'{"ok":false,"err":"pose invalid"}'))
        with patch('m20.platform_core.urlopen',side_effect=response):
            with self.assertRaises(ValueError) as error:rpc('http://offline.invalid','/sensors/autonomy',{'mapping':True})
        self.assertNotIsInstance(error.exception,BridgeTransportError);p._error(error.exception,g)
        self.assertEqual(p.read(g)['state'],'invalidated')

    def test_cancel_immediately_invalidates_and_ignores_late_fetch_result(self):
        entered=threading.Event();release=threading.Event()
        def fetch(context,stop):entered.set();release.wait(1);return self.packet()
        p=LatestObservationPump(fetch,clock=lambda:100.);self.pumps.append(p);g=p.start({})
        self.assertTrue(entered.wait(.5));p.cancel();self.assertEqual(p.read(g)['state'],'cancelled')
        release.set();self.assertTrue(p.join(.5));self.assertIsNone(p.read(g)['observation'])
        self.assertEqual(p.status()['counters']['published'],0)

    def test_old_connection_result_cannot_overwrite_new_generation(self):
        old_entered=threading.Event();old_release=threading.Event();new_done=threading.Event();contexts=[]
        def fetch(context,stop):
            contexts.append(context['robot'])
            if context['robot']=='old':old_entered.set();old_release.wait(1);return self.packet(stamp=500.)
            if not new_done.is_set():new_done.set();return self.packet(stamp=20.)
            stop.wait(1);return self.packet(stamp=20.)
        p=LatestObservationPump(fetch,clock=lambda:100.);self.pumps.append(p)
        source={'robot':'old'};old=p.start(source);self.assertTrue(old_entered.wait(.5));source['robot']='mutated'
        new=p.start({'robot':'new'});self.assertTrue(new_done.wait(.5));old_release.set()
        deadline=time.monotonic()+.5
        while p.status()['counters']['published']<1 and time.monotonic()<deadline:time.sleep(.001)
        self.assertEqual(p.read(old)['state'],'generation_changed')
        self.assertEqual(p.read(new)['observation']['stamp'],20.)
        self.assertNotIn('mutated',contexts)

    def test_new_generation_may_restart_source_clock_without_accepting_old_result(self):
        p,g,c=self.pump();p._publish(self.packet(stamp=900.),g)
        newer=p.start({'robot':'restarted'});self.assertTrue(p._publish(self.packet(stamp=1.),newer))
        self.assertFalse(p._publish(self.packet(stamp=999.),g))
        self.assertEqual(p.read(newer)['observation']['stamp'],1.)

    def test_reader_mutation_cannot_change_cached_geometry_or_deadline(self):
        p,g,c=self.pump();p._publish(self.packet(),g)
        read=p.read(g);read['observation']['floor'].append([9,9]);read['observation']['_valid_until']=200.
        again=p.read(g);self.assertEqual(again['observation']['floor'],[[5,0]])
        self.assertEqual(again['observation']['_valid_until'],100.3)


if __name__=='__main__':unittest.main()
