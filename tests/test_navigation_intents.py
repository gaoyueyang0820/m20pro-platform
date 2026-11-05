"""Synthetic cancellation ordering contracts; no robot, socket or file I/O."""
import unittest
from m20.navigation_intents import NavigationIntents


class IntentLedgerTests(unittest.TestCase):
    def setUp(self):
        self.ledger=NavigationIntents()
        self.req=dict(client_intent='synthetic-intent-0001',server_instance='server',connection_generation=4)
    def test_cancel_before_delayed_start_remains_revoked(self):
        token=self.ledger.revoke(self.req,'server',4)
        with self.assertRaisesRegex(ValueError,'撤销'):self.ledger.register(self.req,'server',4)
        self.assertFalse(self.ledger.valid(token,'server',4))
    def test_same_intent_is_single_use_even_before_cancellation(self):
        token=self.ledger.register(self.req,'server',4)
        self.assertTrue(self.ledger.valid(token,'server',4))
        with self.assertRaisesRegex(ValueError,'已使用'):self.ledger.register(self.req,'server',4)
    def test_cancellation_only_revokes_matching_intent(self):
        first=self.ledger.register(self.req,'server',4)
        other=dict(self.req,client_intent='synthetic-intent-0002')
        second=self.ledger.register(other,'server',4)
        self.ledger.revoke(self.req,'server',4)
        self.assertFalse(self.ledger.valid(first,'server',4));self.assertTrue(self.ledger.valid(second,'server',4))
        nav=self.ledger.nav_fields(second)
        self.assertFalse(self.ledger.owns(nav,first));self.assertTrue(self.ledger.owns(nav,second))
    def test_old_server_or_connection_cannot_register_or_revoke(self):
        for instance,generation in [('new-server',4),('server',5)]:
            with self.subTest(instance=instance,generation=generation):
                with self.assertRaisesRegex(ValueError,'旧服务或旧连接'):self.ledger.register(self.req,instance,generation)
                with self.assertRaisesRegex(ValueError,'旧服务或旧连接'):self.ledger.revoke(self.req,instance,generation)
    def test_reconnecting_discards_previous_scope_without_resurrecting_its_token(self):
        token=self.ledger.register(self.req,'server',4)
        fresh=dict(self.req,connection_generation=5)
        newer=self.ledger.register(fresh,'server',5)
        self.assertFalse(self.ledger.valid(token,'server',5));self.assertTrue(self.ledger.valid(newer,'server',5))
    def test_invalid_or_partial_identity_cannot_be_registered(self):
        for delta in [dict(client_intent='short'),dict(client_intent=None),dict(connection_generation=True),dict(server_instance=None)]:
            with self.assertRaises(ValueError):self.ledger.register(dict(self.req,**delta),'server',4)
    def test_legacy_caller_remains_supported_but_cannot_use_scoped_cancel(self):
        self.assertIsNone(self.ledger.register({},'server',4))
        with self.assertRaises(ValueError):self.ledger.revoke({},'server',4)


if __name__=='__main__':unittest.main()
