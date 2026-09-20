"""Single-image upload regression tests without ROS, camera or network."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import jieguo_uploader as module


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        logger = patch.object(module.base, 'rospy', Mock())
        logger.start()
        self.addCleanup(logger.stop)
        self.u = u = module.SnapshotJieguoUploader.__new__(module.SnapshotJieguoUploader)
        u.results_root = self.temp.name
        u.robot_code = 'DT202600001'
        u.items = [dict(id=0, name='lamp', display_name='车灯破损'),
                   dict(id=1, name='oil', display_name='油箱漏油')]
        u.lock = threading.RLock()
        u.task_active = False
        u.current_task_id = u.current_task_name = u.current_task_identity = ''
        u.record = u.active_collection = None
        u.uploaded_points = set()
        u.recovered_collecting = {}
        u.outbox = Mock()
        u.min_confidence = .35
        u.latest_rtsp_frame = Mock()
        u.snapshot_interval = 0
        def save(collection, stem):
            path = Path(u.task_dir) / (stem + '.jpg')
            path.write_bytes(b'jpeg fixture')
            return str(path)
        u._save_screenshot = save

    def status(self, name='任务甲', point='巡检点2等待', task_type='ACTION_WAIT_TIME'):
        self.u._apply_robot_task(dict(main_task_name=name, task_name=point,
                                     task_state='STATE_DOING', task_type=task_type))

    def collect(self, name='lamp'):
        self.u._collect_frame({'objects': [dict(class_name=name, confidence=.9)]})

    def records(self):
        return [module.read_json(str(path)) for path in Path(self.temp.name).glob('.batch-*.json')]

    def test_flat_independent_images_have_frozen_metadata(self):
        self.status()
        self.collect()
        self.status(point='巡检点3等待')
        self.collect('oil')
        records = sorted(self.records(), key=lambda r: r['createdMs'])
        self.assertEqual(len(records), 2)
        self.assertFalse(any(p.is_dir() for p in Path(self.temp.name).iterdir()))
        self.assertFalse(list(Path(self.temp.name).glob('*.txt')))
        first, second = [r['imageMetadata'] for r in records]
        self.assertEqual(first['taskName'], '任务甲')
        self.assertEqual(first['robotCode'], 'DT202600001')
        self.assertEqual(first['pointName'], '巡检点2')
        self.assertEqual(second['pointName'], '巡检点3')
        self.assertEqual(first['result'], [dict(name='车灯破损', content='')])
        self.assertEqual(second['result'], [dict(name='油箱漏油', content='')])
        self.assertEqual(first['timestamp'], records[0]['createdMs'])
        self.assertEqual(self.u.outbox.enqueue_path.call_count, 2)

    def test_unknown_point_does_not_block_and_task_switch_clears_old_point(self):
        self.status()
        self.collect()
        self.status(name='任务乙', point='等待')
        self.collect()
        self.assertEqual(len(self.records()), 2)
        self.assertEqual(max(self.records(), key=lambda r: r['createdMs'])['imageMetadata']['pointName'], '')
        self.u._point_callback(Mock(data=json.dumps(dict(event='arrived', point_name='巡航点04'))))
        self.collect()
        self.assertEqual(len(self.records()), 3)
        self.u._point_callback(Mock(data=json.dumps(dict(event='leave', point_name='巡航点04'))))
        self.collect()
        self.assertEqual(len(self.records()), 4)
        self.assertEqual(max(self.records(), key=lambda r: r['createdMs'])['imageMetadata']['pointName'], '巡航点04')

    def test_arrival_before_task_and_leave_are_remembered_for_wait(self):
        for event in ('arrived', 'leave'):
            self.u._point_callback(Mock(data=json.dumps(dict(event=event, point_name='巡航点04东侧'))))
        self.status(point='等待20s')
        self.collect()
        self.assertEqual(self.records()[0]['imageMetadata']['pointName'], '巡航点04东侧')

    def test_move_step_point_survives_generic_twenty_second_wait(self):
        self.status(point='巡航点04', task_type='TASK_MOVE_TO')
        self.status(point='等待20s')
        self.collect()
        self.assertEqual(self.records()[0]['imageMetadata']['pointName'], '巡航点04')
        self.status(point='移动到巡航点05', task_type='TASK_MOVE_TO')
        self.status(point='等待20s')
        self.collect('oil')
        self.assertEqual(max(self.records(), key=lambda r: r['createdMs'])['imageMetadata']['pointName'], '巡航点05')
        self.assertEqual(self.u.point_history, ['巡航点04', '巡航点05'])

    def test_old_pending_metadata_converts_to_result(self):
        record = dict(imageMetadata=dict(detectionItems=[dict(name='车灯破损', count=1)],
                                        pointName='巡航点2', content='old'))
        metadata = module.image_upload_metadata(record)
        self.assertEqual(metadata['result'], [dict(name='车灯破损', content=1)])
        self.assertNotIn('content', metadata)
        self.assertNotIn('detectionItems', metadata)

    def test_recovery_queues_each_image_separately(self):
        self.status()
        self.collect()
        self.collect('oil')
        self.u.outbox.reset_mock()
        self.u._recover_ready_tasks()
        self.assertEqual(self.u.outbox.enqueue_path.call_count, 2)

    def test_task_code_is_saved_and_not_reused_after_task_switch(self):
        self.u._apply_robot_task(dict(main_task_name='任务甲', taskCode='PLATFORM-007',
                                     task_state='STATE_DOING', task_type='ACTION_WAIT_TIME'))
        self.status()
        self.collect()
        self.assertEqual(self.records()[0]['taskCode'], 'PLATFORM-007')
        self.status(name='任务乙')
        self.collect()
        record = max(self.records(), key=lambda r: r['createdMs'])
        self.assertEqual(record['taskCode'], module.resolve_task_code('任务乙'))
        self.assertNotEqual(record['taskCode'], 'PLATFORM-007')

    def test_task_code_matches_existing_mqtt_generation(self):
        import hashlib
        expected = 'TASK-' + hashlib.sha256('任务8'.encode('utf-8')).hexdigest().upper()[:16]
        self.assertEqual(module.resolve_task_code('任务8'), expected)
        self.assertEqual(module.resolve_task_code('任务8', 'REAL-CODE'), 'REAL-CODE')

    def test_actual_mqtt_publish_contains_result_and_logs_same_wire_message(self):
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.topic = 'thing/robot/DT202600001/file'
        box.client = Mock()
        box.client.publish.return_value.rc = 0
        box.client.publish.return_value.is_published.return_value = True
        for fields, expected in [
            ({'content': None}, []),
            ({'content': '车灯破损:1'}, [{'name': '车灯破损', 'content': '1'}]),
            ({'result': None, 'detectionItems': [{'name': '车灯破损', 'count': 1}]},
             [{'name': '车灯破损', 'content': ''}]),
            ({'result': [{'name': '车灯破损', 'content': 2}]},
             [{'name': '车灯破损', 'content': '2'}]),
        ]:
            payload = dict(method='file_upload_request', tid='test', timestamp=123,
                           data=dict(files=[{'fileName': '任务7.jpg'}], **fields))
            box.publish_confirmed(payload)
            args, kwargs = box.client.publish.call_args
            wire = args[1]
            decoded = json.loads(wire)
            self.assertIsInstance(decoded, dict)  # 不重复编码成JSON字符串
            self.assertEqual(decoded['data']['files'][0]['result'], expected)
            self.assertNotIn('result', decoded['data'])
            self.assertNotIn('content', decoded['data'])
            self.assertNotIn('detectionItems', decoded['data'])
            self.assertEqual(decoded['data']['files'][0]['fileName'], '任务7.jpg')
            self.assertEqual(kwargs, dict(qos=1, retain=False))
            module.base.rospy.loginfo.assert_any_call('MQTT上传请求原文[result-v2]: %s', wire)

    def test_receipt_serialization_preserves_numeric_status(self):
        for code in (0, -1):
            payload = dict(method='file_upload_reply', data={'objectKey': 'key', 'result': code})
            self.assertEqual(json.loads(module.serialize_upload_message(payload)), payload)
        with self.assertRaises(ValueError):
            module.serialize_upload_message(dict(method='file_upload_reply', data={'objectKey': 'key', 'result': []}))

    def test_request_and_receipt_keep_metadata_and_original_protocol(self):
        self.status()
        self.collect()
        record = self.records()[0]
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.robot_code = 'DT202600001'
        box.condition = threading.Condition()
        box.pending = {}
        box.stopping = False
        def publish(payload):
            if payload['method'] == 'file_upload_request':
                self.assertEqual(payload['data']['files'][0]['pointName'], '巡检点2')
                self.assertEqual(payload['data']['timestamp'], record['createdMs'])
            if payload['method'] == 'file_upload_request':
                self.assertEqual(payload['data']['stationCode'], record['taskCode'])
                self.assertNotEqual(payload['data']['stationCode'], record['taskId'])
                self.assertEqual(payload['data']['files'][0]['result'], [dict(name='车灯破损', content='')])
                self.assertNotIn('content', payload['data'])
                self.assertNotIn('detectionItems', payload['data'])
                self.assertEqual(payload['data']['files'][0]['fileName'], record['images'][0])
                self.assertNotIn('result', payload['data'])
                box.pending[payload['tid']] = dict(tid=payload['tid'], data=dict(result=0))
            else:
                self.assertEqual(payload['data']['result'], 0)
                self.assertNotIn('resultCode', payload['data'])
        box.publish_confirmed = publish
        box.request(record, [{'fileName': record['images'][0]}])
        name = record['images'][0]
        record['receipts'] = {name: dict(tid='reply', objectKey='key')}
        box.send_file_receipt(record, str(next(Path(self.temp.name).glob('.batch-*.json'))), name)
        self.assertTrue(record['receipts'][name]['acknowledged'])

    def test_actual_receipt_publish_contains_saved_recognition_result(self):
        self.status()
        self.collect()
        record = self.records()[0]
        name = record['images'][0]
        record['receipts'] = {name: dict(tid='receipt-test', objectKey='object-key')}
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.topic = 'thing/robot/DT202600001/file'
        box.client = Mock()
        box.client.publish.return_value.rc = 0
        box.client.publish.return_value.is_published.return_value = True
        path = str(next(Path(self.temp.name).glob('.batch-*.json')))
        box.send_file_receipt(record, path, name)
        wire = box.client.publish.call_args[0][1]
        payload = json.loads(wire)
        self.assertEqual(payload['method'], 'file_upload_reply')
        self.assertEqual(payload['data']['result'], 0)
        self.assertNotIn('resultCode', payload['data'])
        self.assertEqual(payload['data']['objectKey'], 'object-key')
        self.assertEqual(payload['data'], {'objectKey': 'object-key', 'result': 0})
        self.assertNotIn('content', payload['data'])
        module.base.rospy.loginfo.assert_any_call('MQTT车体回执原文[result-v3]: %s', wire)

    def test_file_results_stay_with_each_image_and_preserve_plate_text(self):
        files = [
            dict(fileName='one.jpg', pointName='巡航点1',
                 result=[dict(name='轮胎掩护', content=''), dict(name='车牌号', content='B1001')]),
            dict(fileName='two.jpg', pointName='巡航点2',
                 result=[dict(name='灭火器', content='')]),
        ]
        payload = dict(method='file_upload_request', data=dict(files=files))
        decoded = json.loads(module.serialize_upload_message(payload))
        self.assertEqual(decoded['data']['files'], files)
        self.assertNotIn('result', decoded['data'])
        self.assertNotIn('pointName', decoded['data'])

    def test_receipt_publish_failure_stays_pending_and_can_be_retried(self):
        self.status()
        self.collect()
        path = str(next(Path(self.temp.name).glob('.batch-*.json')))
        record = module.read_json(path)
        name = record['images'][0]
        record['receipts'] = {name: dict(tid='same-tid', objectKey='key', acknowledged=False)}
        module.base.atomic_write_json(path, record)
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.publish_confirmed = Mock(side_effect=RuntimeError('no PUBACK'))
        with self.assertRaises(RuntimeError):
            box.send_file_receipt(record, path, name)
        saved = module.read_json(path)
        self.assertFalse(saved['receipts'][name]['acknowledged'])
        self.assertNotIn(name, saved.get('done', []))
        box.publish_confirmed.side_effect = None
        box.send_file_receipt(saved, path, name)
        self.assertTrue(module.read_json(path)['receipts'][name]['acknowledged'])
        self.assertEqual(box.publish_confirmed.call_args[0][0]['data'], {'objectKey': 'key', 'result': 0})
        box.publish_confirmed.reset_mock()
        box.send_file_receipt(saved, path, name)
        box.publish_confirmed.assert_not_called()

    def test_failure_receipt_reports_failure_without_affecting_retry(self):
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.publish_confirmed = Mock()
        box.send_file_failure('tid-1', 'key-1')
        payload = box.publish_confirmed.call_args[0][0]
        self.assertEqual(payload['method'], 'file_upload_reply')
        self.assertEqual(payload['tid'], 'tid-1')
        self.assertEqual(payload['data'], {'objectKey': 'key-1', 'result': -1})
        box.publish_confirmed.side_effect = RuntimeError('offline')
        box.send_file_failure('tid-1', 'key-1')  # 通知失败不能中断原图片重试。

    def test_legacy_multi_image_summary_is_not_assigned_to_each_image(self):
        payload = dict(method='file_upload_request', data=dict(
            files=[dict(fileName='one.jpg'), dict(fileName='two.jpg')],
            result=[dict(name='车灯破损', content='')], pointName='巡航点1'))
        decoded = json.loads(module.serialize_upload_message(payload))
        for file in decoded['data']['files']:
            self.assertEqual(file['result'], [])
            self.assertEqual(file['pointName'], '')

    def test_snapshot_preserves_available_recognition_text(self):
        self.status()
        self.u._collect_frame({'objects': [dict(class_name='lamp', confidence=.9, content='B1001')]})
        self.assertEqual(self.records()[0]['imageMetadata']['result'],
                         [dict(name='车灯破损', content='B1001')])

    def test_failed_image_cleanup_cannot_remove_neighbor(self):
        self.status()
        self.collect()
        self.collect('oil')
        paths = list(Path(self.temp.name).glob('.batch-*.json'))
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.results_root = self.temp.name
        box._delete_failed_task(str(paths[0]), module.read_json(str(paths[0])))
        self.assertTrue(paths[1].exists())
        self.assertEqual(len(list(Path(self.temp.name).glob('*.jpg'))), 1)

    def test_confirmed_image_cleanup_preserves_root_and_neighbor(self):
        self.status()
        self.collect()
        self.collect('oil')
        paths = list(Path(self.temp.name).glob('.batch-*.json'))
        record = module.read_json(str(paths[0]))
        record['uploaded'] = True
        module.base.atomic_write_json(str(paths[0]), record)
        box = module.FileOutbox.__new__(module.FileOutbox)
        box.results_root = self.temp.name
        box.upload(str(paths[0]))
        self.assertTrue(Path(self.temp.name).is_dir())
        self.assertTrue(paths[1].exists())
        self.assertFalse(paths[0].exists())
        self.assertEqual(len(list(Path(self.temp.name).glob('*.jpg'))), 1)


if __name__ == '__main__':
    unittest.main()
