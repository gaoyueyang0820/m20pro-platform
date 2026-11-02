"""A discarded capture is finished work, not a permanent navigation blocker."""
import unittest
import test_platform as cases


class AfterDiscardNavigationTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    live=cases.PlatformTests.live

    def test_start_patrol_after_discard_accepts_finished_capture_state(self):
        route=cases.PlatformTests.route(self,live=True)
        self.app.hub.get=lambda *args,**kwargs:dict(phase='discarded',in_progress=False)
        self.app.start_route(dict(id=route['id']))
        self.assertEqual(self.app.nav['status'],'running')
        self.assertNotEqual(self.app.nav.get('phase'),'validation_failed')

    def test_resume_patrol_after_discard_accepts_finished_capture_state(self):
        route=cases.PlatformTests.route(self,live=True)
        self.app.hub.get=lambda *args,**kwargs:dict(phase='discarded',in_progress=False)
        self.app.start_route(dict(id=route['id']))
        self.app.nav.update(status='paused',phase='paused')
        self.app.resume_route()
        self.assertEqual(self.app.nav['status'],'running')


if __name__=='__main__':unittest.main()
