"""Offline contract tests: no ROS, broker, or vehicle connection."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'mqtt_quick_cruise_receiver.py'
spec = importlib.util.spec_from_file_location('receiver', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReceiverTests(unittest.TestCase):
    def setUp(self):
        self.r = r = module.Receiver.__new__(module.Receiver)
        r.args = SimpleNamespace(robot_code='TEST', task_name='')
        r.lock = threading.RLock()
        r.tasks = [{'name': '一区任务0'}]
        r.status = {'state': 'IDLE', 'data': {'robot_task': {'task_state': 'STATE_IDLE'}}}
        r.catalog_at = r.status_at = time.monotonic()
        r.last_dispatch_at = 0
        r.pending = None
        r.command_topic = 'thing/robot/TEST/services'
        r.reply_topic = 'thing/robot/TEST/services_reply'
        r.db = sqlite3.connect(':memory:')
        r.db.execute('CREATE TABLE commands (tid TEXT PRIMARY KEY, reply TEXT NOT NULL)')
        r.pub = Mock()
        r.pub.get_num_connections.return_value = 1
        r.client = Mock()
        r.ros = Mock()
        r.String = SimpleNamespace
        self.addCleanup(r.db.close)

    def send(self, tid='one', retain=False, **fields):
        msg = {'tid': tid, 'method': 'quick_cruise', 'timestamp': time.time() * 1000,
               'data': {'robotCode': 'TEST'}}
        msg.update(fields)
        self.r.message(self.r.client, None, SimpleNamespace(
            topic=self.r.command_topic, retain=retain, payload=json.dumps(msg).encode()))

    def reply(self, state='success', source='robot_bridge', command_id='one'):
        self.r.task_reply(SimpleNamespace(data=json.dumps({
            'command_id': command_id, 'command_status': state, 'source': source})))

    def result(self):
        return json.loads(self.r.client.publish.call_args.args[1])['data']['result']

    def test_start_waits_for_matching_bridge_reply(self):
        self.send()
        outgoing = json.loads(self.r.pub.publish.call_args.args[0].data)
        self.assertEqual(outgoing['action'], 'inspection.start')
        self.assertEqual(outgoing['tid'], 'one')
        self.assertEqual(outgoing['data'], {'task_name': '一区任务0', 'loop_time': 1})
        self.r.client.publish.assert_not_called()
        self.reply(source='other')
        self.reply(command_id='other')
        self.r.client.publish.assert_not_called()
        self.reply()
        self.assertEqual(self.result(), 0)

    def test_duplicate_pending_and_completed(self):
        self.send()
        self.send()
        self.reply()
        self.send()
        self.assertEqual(self.r.pub.publish.call_count, 1)
        self.assertEqual(self.result(), 0)

    def test_persisted_command_is_not_reissued_after_restart(self):
        self.send()
        self.r.pending = None  # Recovered database, no in-memory pending request.
        self.send()
        self.assertEqual(self.r.pub.publish.call_count, 1)
        self.assertEqual(self.result(), 1)

    def test_retained_wrong_robot_and_other_method_ignored(self):
        self.send(retain=True)
        self.send(data={'robotCode': 'OTHER'})
        self.send(method='task_upload')
        self.r.pub.publish.assert_not_called()
        self.r.client.publish.assert_not_called()

    def test_invalid_timestamps(self):
        for i, stamp in enumerate([0, None, True, float('nan'), float('inf')]):
            self.send(tid=str(i), timestamp=stamp)
            self.assertEqual(self.result(), 1)
        self.r.pub.publish.assert_not_called()

    def test_stale_status_and_catalog(self):
        self.r.status_at -= 11
        self.send()
        self.assertEqual(self.result(), 1)
        self.r.status_at = time.monotonic()
        self.r.catalog_at -= 31
        self.send(tid='two')
        self.assertEqual(self.result(), 1)
        self.r.pub.publish.assert_not_called()

    def test_busy_rejected(self):
        self.r.status['state'] = 'MOVING'
        self.send()
        self.assertEqual(self.result(), 1)
        self.r.pub.publish.assert_not_called()

    def test_missing_subscriber(self):
        self.r.pub.get_num_connections.return_value = 0
        self.send()
        self.assertEqual(self.result(), 1)
        self.r.pub.publish.assert_not_called()

    def test_failure_and_timeout(self):
        self.send()
        self.reply(state='failed')
        self.assertEqual(self.result(), 1)
        self.assertIsNone(self.r.pending)
        self.r.status_at = time.monotonic()
        self.send(tid='two')
        self.r.pending = ('two', time.monotonic() - 31)
        self.r.check_timeout(None)
        self.assertEqual(self.result(), 1)
        self.assertIsNone(self.r.pending)

    def test_new_tid_cannot_use_pre_dispatch_idle_status(self):
        self.send()
        self.reply()
        self.send(tid='two')
        self.assertEqual(self.result(), 1)
        self.assertEqual(self.r.pub.publish.call_count, 1)

    def test_task_selection(self):
        self.assertEqual(module.choose_task([None, {'name': None}, '任务01', '任务0']), '任务0')
        with self.assertRaises(ValueError):
            module.choose_task(['一区任务0', '二区任务0'])
        self.assertEqual(module.choose_task(['一区任务0', '二区任务0'], '二区任务0'), '二区任务0')
        with self.assertRaises(ValueError):
            module.choose_task(['任务01'])

    def test_wrapped_json(self):
        self.assertEqual(module.decode(json.dumps(json.dumps({'tid': 'one'}))), {'tid': 'one'})


if __name__ == '__main__':
    unittest.main()
