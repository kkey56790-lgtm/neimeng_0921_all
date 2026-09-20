"""Task archive regression tests; no ROS, camera or network required."""
from pathlib import Path
import sys
import tempfile
import collections
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import jieguo_uploader as module


class TaskArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        logger = patch.object(module.base, 'rospy', Mock())
        logger.start()
        self.addCleanup(logger.stop)
        self.u = u = module.JieguoUploader.__new__(module.JieguoUploader)
        u.results_root = self.temp.name
        u.items = [{'id': 0, 'name': 'truck', 'display_name': '卡车'}]
        u.robot_code = 'TEST'
        u.lock = threading.RLock()
        u.task_active = False
        u.current_task_id = u.current_task_name = u.current_task_identity = ''
        u.task_dir = ''
        u.record = u.active_collection = None
        u.uploaded_points = set()
        u.recovered_collecting = {}
        u.collection_paused = False
        u.outbox = Mock()
        u.min_confidence = .35
        u.latest_rtsp_frame = None
        u.snapshot_interval = 3

    def status(self, state='STATE_DOING', name='一区巡检', **fields):
        return self.u._apply_robot_task(dict(
            task_state=state, main_task_name=name, **fields))

    def test_subtasks_and_pause_share_directory_until_queue_finishes(self):
        self.status(task_name='到点1', task_type='TASK_MOVE_TO', task_time=1)
        path = self.u.record_path
        for state, fields in [
            ('STATE_DOING', {'task_name': '暂停1', 'task_type': 'ACTION_WAIT_TIME', 'task_time': 20}),
            ('STATE_PAUSE', {'task_time': 30}),
            ('STATE_PAUSED', {'task_time': 40}),
            ('STATE_FINISH', {'queue_size': 1}),
            ('STATE_DOING', {'task_name': '到点2', 'task_time': 50}),
        ]:
            self.status(state, **fields)
            self.assertEqual(self.u.record_path, path)
            self.assertFalse(self.u.collection_paused)
            self.u.outbox.enqueue_path.assert_not_called()
        self.status('STATE_FINISH', queue_size=0)
        self.u.outbox.enqueue_path.assert_called_once_with(path)
        self.assertEqual(module.read_json(path)['state'], 'READY')
        self.status('STATE_FINISH', queue_size=0)
        self.u.outbox.enqueue_path.assert_called_once()

    def test_pause_still_collects_detection(self):
        self.status('STATE_PAUSE')
        self.u._collect_frame({'objects': [{'class_name': 'truck', 'confidence': .9}]})
        self.assertEqual(module.read_json(self.u.record_path)['counts']['truck'], 1)
        self.u.outbox.enqueue_path.assert_not_called()

    def test_same_name_next_run_has_new_directory(self):
        self.status()
        first = self.u.task_dir
        self.status('STATE_FINISH')
        self.status()
        self.assertNotEqual(first, self.u.task_dir)

    def test_detection_screenshots_stay_local_until_end(self):
        self.status()
        directory = self.u.task_dir
        self.u.latest_rtsp_frame = Mock()

        def save(collection, stem):
            path = Path(self.u.task_dir) / (stem + '.jpg')
            path.write_bytes(b'offline image fixture')
            return str(path)

        self.u._save_screenshot = save
        for state in ('STATE_DOING', 'STATE_PAUSE'):
            self.status(state)
            self.u.last_snapshot = 0
            self.u._collect_frame({'objects': [{'class_name': 'truck', 'confidence': .9}]})
        self.assertEqual(len(self.u.record['images']), 2)
        for name in self.u.record['images']:
            self.assertTrue((Path(directory) / name).is_file())
        self.u.outbox.enqueue_path.assert_not_called()
        self.status('STATE_FINISH')
        record = module.read_json(self.u.record_path)
        self.assertEqual(len(record['images']), 2)
        self.assertEqual(len(record['hashes']), 3)
        self.u.outbox.enqueue_path.assert_called_once()

    def test_switch_archives_old_name(self):
        self.status()
        old = self.u.record_path
        self.status(name='二区巡检')
        self.assertEqual(module.read_json(old)['taskName'], '一区巡检')
        self.assertEqual(self.u.record['taskName'], '二区巡检')
        self.u.outbox.enqueue_path.assert_called_once_with(old)

    def simulate_restart(self):
        self.u.record = self.u.active_collection = None
        self.u.task_active = False
        self.u.task_dir = ''
        self.u._recover_ready_tasks()

    def test_restart_resumes_and_preserves_original_name(self):
        name = '一区/巡检:' + '长名称' * 35
        self.status(name=name)
        path = self.u.record_path
        self.simulate_restart()
        self.u.outbox.enqueue_path.assert_not_called()
        self.status(name=name, task_time=999)
        self.assertEqual(self.u.record_path, path)
        self.assertEqual(self.u.record['taskName'], name)
        self.status('STATE_FINISH', name=name)
        self.u.outbox.enqueue_path.assert_called_once_with(path)

    def test_restart_waits_for_completion(self):
        self.status()
        path = self.u.record_path
        self.simulate_restart()
        self.u.outbox.enqueue_path.assert_not_called()
        self.status('STATE_FINISH', queue_size=2)
        self.u.outbox.enqueue_path.assert_not_called()
        self.status('STATE_FINISH', queue_size=0)
        self.u.outbox.enqueue_path.assert_called_once_with(path)

    def test_completed_batch_recovered_immediately(self):
        self.status()
        path = self.u.record_path
        self.status('STATE_FINISH')
        self.u.outbox.reset_mock()
        self.simulate_restart()
        self.u.outbox.enqueue_path.assert_called_once_with(path)

    def test_upload_request_and_receipt_carry_task_metadata(self):
        self.status()
        record = self.u.record
        outbox = module.FileOutbox.__new__(module.FileOutbox)
        outbox.robot_code = 'TEST'
        outbox.condition = threading.Condition()
        outbox.pending = {}
        outbox.stopping = False
        outbox.response_timeout = 1
        payloads = []

        def publish(payload):
            payloads.append(payload)
            if payload['method'] == 'file_upload_request':
                outbox.pending[payload['tid']] = {'tid': payload['tid'], 'data': {'result': 0}}

        outbox.publish_confirmed = publish
        outbox.request(record, [{'fileName': 'test.jpg'}])
        record['receipts'] = {'test.jpg': {'tid': 'receipt', 'objectKey': 'key'}}
        outbox.send_file_receipt(record, self.u.record_path, 'test.jpg')
        self.assertEqual(payloads[0]['data']['taskName'], '一区巡检')
        self.assertEqual(payloads[0]['data']['taskId'], record['taskId'])
        self.assertEqual(payloads[1]['data'], {'objectKey': 'key', 'result': 0})

    def failure_outbox(self):
        outbox = module.FileOutbox.__new__(module.FileOutbox)
        outbox.results_root = self.temp.name
        outbox.stopping = False
        outbox.condition = threading.Condition()
        outbox.items = collections.deque()
        outbox.upload = Mock(side_effect=RuntimeError('credential timeout'))
        return outbox

    def test_third_failure_deletes_only_failed_task_and_persists_count(self):
        self.status()
        path, directory = self.u.record_path, self.u.task_dir
        (Path(directory) / 'image.jpg').write_bytes(b'image')
        self.status('STATE_FINISH')
        self.status(name='下一任务')
        active_directory = self.u.task_dir
        outbox = self.failure_outbox()
        for count in (1, 2):
            with self.assertRaises(RuntimeError):
                outbox.upload_with_retry_limit(path)
            self.assertEqual(module.read_json(path)['uploadFailures'], count)
            self.assertTrue(Path(directory).exists())
        # 重启不重置失败次数；第三次失败清理整个目录，包括未跟踪的图片。
        outbox = self.failure_outbox()
        outbox.items.extend([path, self.u.record_path])
        outbox.upload_with_retry_limit(path)
        self.assertFalse(Path(directory).exists())
        self.assertTrue(Path(active_directory).exists())
        self.assertEqual(list(outbox.items), [self.u.record_path])

    def test_pending_deletion_does_not_make_fourth_upload_attempt(self):
        self.status()
        self.status('STATE_FINISH')
        path = self.u.record_path
        record = module.read_json(path)
        record['uploadFailures'] = 3
        module.base.atomic_write_json(path, record)
        outbox = self.failure_outbox()
        outbox.upload_with_retry_limit(path)
        outbox.upload.assert_not_called()
        self.assertFalse(Path(path).parent.exists())

    def test_retry_success_is_not_deleted_by_failure_policy(self):
        self.status()
        self.status('STATE_FINISH')
        outbox = self.failure_outbox()
        with self.assertRaises(RuntimeError):
            outbox.upload_with_retry_limit(self.u.record_path)
        outbox.upload.side_effect = None
        outbox.upload_with_retry_limit(self.u.record_path)
        self.assertTrue(Path(self.u.record_path).exists())
        self.assertEqual(module.read_json(self.u.record_path)['uploadFailures'], 1)

    def test_deletion_refuses_result_root_and_collecting_directory(self):
        self.status()
        outbox = self.failure_outbox()
        record = dict(self.u.record, uploadFailures=3)
        with self.assertRaises(RuntimeError):
            outbox._delete_failed_task(self.u.record_path, record)
        with self.assertRaises(RuntimeError):
            outbox._delete_failed_task(str(Path(self.temp.name) / '.batch-root.json'),
                                       dict(record, state='READY'))
        self.assertTrue(Path(self.u.record_path).exists())

    def test_shutdown_does_not_count_as_failure(self):
        self.status()
        self.status('STATE_FINISH')
        outbox = self.failure_outbox()
        outbox.stopping = True
        with self.assertRaises(RuntimeError):
            outbox.upload_with_retry_limit(self.u.record_path)
        self.assertNotIn('uploadFailures', module.read_json(self.u.record_path))


if __name__ == '__main__':
    unittest.main()
