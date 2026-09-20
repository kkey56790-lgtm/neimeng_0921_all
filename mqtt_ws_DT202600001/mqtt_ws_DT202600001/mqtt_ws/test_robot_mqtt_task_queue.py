"""Offline regression tests. No MQTT broker or robot is contacted."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.dont_write_bytecode = True
fake_modules = {name: types.ModuleType(name) for name in
                ('paho', 'paho.mqtt', 'paho.mqtt.client', 'requests')}
fake_modules['paho.mqtt.client'].Client = Mock
fake_modules['paho.mqtt.client'].MQTTv311 = 4
path = Path(__file__).with_name('robot_mqtt_all_in_one_task_queue.py')
spec = importlib.util.spec_from_file_location('queue_service_under_test', path)
m = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, fake_modules):
    spec.loader.exec_module(m)


class QueueTests(unittest.TestCase):
    def setUp(self):
        self.s = m.QueuedRobotMQTTService()
        self.catalog = patch.object(m, 'read_task_catalog', return_value=[{'name': 'A'}, {'name': 'B'}]).start()
        self.status = patch.object(m, 'get_task_status', return_value={}).start()
        self.start = patch.object(m, 'start_robot_task', return_value={'result': True}).start()
        self.stop = patch.object(m, 'stop_robot_task', return_value=True).start()
        self.release = patch.object(m, 'wait_task_release', return_value=True).start()
        self.addCleanup(patch.stopall)


    def data(self, name):
        return {'robotCode': m.ROBOT_CODE, 'mainTaskName': name, 'taskCode': m.generate_task_code(name)}

    def upload(self, name='A', tid='one'):
        self.s._dispatch(tid, 'task_upload', self.data(name))

    def tick(self, state, name='', stamp='t0'):
        self.s.next_task_poll = 0
        self.status.return_value = {'task_state': state, 'main_task_name': name, 'task_time': stamp}
        self.s._advance_tasks()

    def messages(self):
        return [entry[0] for entry in self.s.reply_outbox]

    def test_sequential_start_finish_exact_wire_format(self):
        self.upload('A','one'); self.upload('B','two')
        self.assertFalse(self.messages())
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.tick('STATE_DOING','A','t2'); self.tick('STATE_FINISH','A','t3')
        self.tick('STATE_FINISH','A','t3'); self.tick('STATE_DOING','B','t4')
        self.tick('STATE_FINISH','B','t5')
        self.assertEqual([c.args[0] for c in self.start.call_args_list],['A','B'])
        self.assertEqual([x['data'] for x in self.messages()], [
            {'result':0}, {'mainTaskName':'A','result':0},
            {'result':0}, {'mainTaskName':'B','result':0}])
        self.assertEqual([x['tid'] for x in self.messages()],['one','one','two','two'])
        for x in self.messages():
            self.assertEqual(set(x), {'tid','method','timestamp','data'})
            self.assertEqual(x['method'],'task_upload')
        self.stop.assert_not_called()

    def test_actual_publish_does_not_inject_reply_fields(self):
        self.s.connected.set()
        info=Mock(rc=0); info.is_published.return_value=True
        self.s.client.publish.return_value=info
        self.upload(); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._flush_replies()
        call=self.s.client.publish.call_args
        self.assertEqual(call.args[0],m.REPLY_TOPIC)
        self.assertEqual(json.loads(call.args[1])['data'],{'result':0})

    def test_task_list_exact_fields(self):
        self.catalog.return_value=[{'name':'A','tasks':[{'name':'POINT','map':'M'}, {'map':'M'}]}]
        self.status.return_value={'main_task_name':'A','task_name':'POINT','task_state':'STATE_DOING',
            'task_time':'t1','local_state':True,'map':'M','obstacle':False,'task_type':'TASK_MOVE_TO',
            'error_info':'','queue_size':1,'remai_distance':1,'remai_loop':0,'remai_time':2,'task_arg':{'name':'POINT'}}
        msg=m.build_task_message()
        self.assertEqual(set(msg['data']),{'robotCode','robotName','taskList','currentTask','taskListError'})
        self.assertEqual(msg['data']['taskList'],[{'mainTaskName':'A','taskCode':m.generate_task_code('A'),
            'index':1,'mapNames':['M'],'subTaskCount':2}])
        self.assertEqual(set(msg['data']['currentTask']),{'main_task_name','task_name','task_state','task_time',
            'local_state','map','obstacle','task_type','error_info','queue_size','remai_distance','remai_loop','remai_time','taskCode'})
        self.s.publish_json(m.OSD_TOPIC,msg)
        wire=json.loads(self.s.client.publish.call_args.args[1])
        self.assertEqual(wire,msg)

    def test_catalog_error_in_task_list_error(self):
        self.catalog.side_effect=m.TaskCatalogError('OFFLINE','offline')
        msg=m.build_task_message()
        self.assertEqual(msg['data']['taskList'],[])
        self.assertIn('offline',msg['data']['taskListError'])

    def test_arrays_and_missing_fields_rejected(self):
        invalid=[{'robotCode':m.ROBOT_CODE,'tasks':[self.data('A')]},
                 dict(self.data('A'),taskCode=[m.generate_task_code('A')]),
                 {'robotCode':m.ROBOT_CODE,'mainTaskName':'A'},
                 dict(self.data('A'),mainTaskName='')]
        for i,data in enumerate(invalid):
            self.s._dispatch(str(i),'task_upload',data)
            self.assertEqual(self.messages()[-1]['data'],{'result':-1})
        self.assertFalse(self.s.waiting_tasks)

    def test_extra_metadata_allowed_without_changing_reply(self):
        self.s._dispatch('extra','task_upload',dict(self.data('A'),taskId='platform-id',task_code='A'))
        self.assertEqual(len(self.s.waiting_tasks),1)
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.assertEqual(self.messages()[-1]['data'],{'result':0})

    def test_missing_field_diagnostic(self):
        with self.assertRaisesRegex(ValueError, 'taskCode'):
            m.parse_task_batch({'robotCode':m.ROBOT_CODE,'mainTaskName':'A'})

    def test_name_and_code_must_match(self):
        self.s._dispatch('id','task_upload',dict(self.data('A'),taskCode=m.generate_task_code('B')))
        self.assertEqual(self.messages()[-1]['data'],{'result':-1})
        self.assertFalse(self.s.waiting_tasks)

    def test_duplicate_never_restarts_replays_completion(self):
        self.upload(); self.upload()
        self.assertEqual(len(self.s.waiting_tasks),1)
        self.assertFalse(self.messages())
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1'); self.tick('STATE_FINISH','A','t2')
        completed=self.messages()[-1]
        self.upload()
        self.assertEqual(self.messages()[-1],completed)
        self.start.assert_called_once_with('A')

    def test_status_unknown_does_not_publish_false_failure(self):
        self.upload(); self.upload('B','two')
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s.active_task['last_valid']=-100
        self.tick('')
        self.assertEqual(self.s.active_task['stage'],'status_unknown')
        self.assertEqual(len(self.messages()),1)
        self.tick('STATE_DOING','A','t2'); self.tick('STATE_FINISH','A','t3')
        self.assertEqual(self.messages()[-1]['data'],{'mainTaskName':'A','result':0})

    def test_stop_clears_pending(self):
        self.upload(); self.upload('B','two'); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._dispatch('stop','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertFalse(self.s.waiting_tasks); self.assertIsNone(self.s.active_task)
        self.assertEqual(self.messages()[-1]['method'],'task_stop')
        self.assertEqual(self.messages()[-1]['data'],{'result':0})

    def test_repeated_fixed_tid_stop_executes_every_time(self):
        for _ in range(2):
            self.s._dispatch('cmd-uuid-004','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertEqual(self.stop.call_count,2)
        self.assertEqual(self.release.call_count,2)
        self.assertEqual(len(self.messages()),2)
        self.assertTrue(all(x['data']=={'result':0} for x in self.messages()))

    def test_stop_can_reuse_upload_tid_without_corrupting_cache(self):
        self.upload('A','same'); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._dispatch('same','task_stop',{'robotCode':m.ROBOT_CODE})
        self.stop.assert_called_once()
        self.assertEqual(self.s.command_history['same']['wire_reply']['method'],'task_upload')
        self.assertEqual(self.s.command_history['same']['wire_reply']['data'],{'mainTaskName':'A','result':-1})
        self.upload('A','same')
        self.assertFalse(self.s.waiting_tasks)

    def test_stop_retry_returns_fresh_result_and_clears_new_tasks(self):
        self.stop.side_effect=[False,True]
        self.s._dispatch('fixed','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertTrue(self.s.queue_blocked)
        self.s._dispatch('fixed','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertFalse(self.s.queue_blocked)
        self.assertEqual([x['data']['result'] for x in self.messages()],[-1,0])
        self.stop.side_effect=None; self.stop.return_value=True
        self.upload('A','new')
        self.s._dispatch('fixed','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertFalse(self.s.waiting_tasks)

    def test_stop_wrong_robot_still_rejected(self):
        self.s._dispatch('fixed','task_stop',{'robotCode':'OTHER'})
        self.stop.assert_not_called()
        self.assertEqual(self.messages()[-1]['data'],{'result':-1})

    def test_failure_cancels_waiting(self):
        self.upload(); self.upload('B','two'); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.tick('STATE_FAIL','A','t2')
        self.assertEqual([x['data'] for x in self.messages()][-2:],[{'mainTaskName':'A','result':-1},{'mainTaskName':'B','result':-1}])
        self.assertFalse(self.s.waiting_tasks)

    def test_start_timeout_not_resent(self):
        self.upload(); self.start.return_value=None; self.tick('STATE_CANCEL')
        self.s.active_task['start_time']=-100; self.tick(''); self.tick('')
        self.start.assert_called_once_with('A'); self.assertFalse(self.messages())
        self.tick('STATE_DOING','A','t1'); self.assertEqual(self.messages()[-1]['data'],{'result':0})

    def test_old_finish_not_used_for_new_run(self):
        self.upload(); self.tick('STATE_FINISH','A','old'); self.tick('STATE_FINISH','A','old')
        self.assertFalse(self.messages())
        self.tick('STATE_DOING','A','new'); self.tick('STATE_FINISH','A','newer')
        self.assertEqual(len(self.messages()),2)

    def test_external_task_not_stopped(self):
        self.upload(); self.tick('STATE_DOING','OTHER','t1')
        self.start.assert_not_called(); self.stop.assert_not_called()

    def test_custom_query_no_longer_subscribed(self):
        msg={'tid':'q','method':'get_current_map_tasks','data':{'robotCode':m.ROBOT_CODE}}
        self.s.on_message(None,None,types.SimpleNamespace(retain=False,topic=m.SERVICE_TOPIC,payload=json.dumps(msg).encode()))
        self.assertTrue(self.s.control_queue.empty())

    def test_offline_replies_retained_until_ack(self):
        self.upload(); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._flush_replies(); self.s.client.publish.assert_not_called()
        self.s.connected.set(); info=Mock(rc=0); info.is_published.return_value=False
        self.s.client.publish.return_value=info
        self.s._flush_replies(); self.s._flush_replies(); self.s.client.publish.assert_called_once()
        info.is_published.return_value=True; self.s._flush_replies()
        self.assertFalse(self.s.reply_outbox)

    def test_switch_preempts_active_preserves_pending(self):
        self.upload('A','one'); self.upload('B','two')
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._dispatch('switch','task_switch',self.data('A'))
        self.stop.assert_called_once()
        self.assertEqual([t['tid'] for t in self.s.waiting_tasks],['switch','two'])
        self.assertEqual(self.messages()[-1]['data'],{'mainTaskName':'A','result':-1})
        self.tick('STATE_CANCEL','A','t1')
        self.tick('STATE_DOING','A','t2'); self.tick('STATE_FINISH','A','t3')
        self.assertEqual(self.messages()[-1]['method'],'task_switch')
        self.assertEqual(self.messages()[-1]['data'],{'mainTaskName':'A','result':0})
        self.tick('STATE_FINISH','A','t3')
        self.assertEqual([c.args[0] for c in self.start.call_args_list],['A','A','B'])

    def test_invalid_switch_does_not_stop(self):
        self.upload(); self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s._dispatch('switch','task_switch',self.data('MISSING'))
        self.stop.assert_not_called()
        self.assertEqual(self.s.active_task['mainTaskName'],'A')

    def test_switch_stop_failure_blocks_and_preserves_pending(self):
        self.upload(); self.upload('B','two'); self.tick('STATE_CANCEL')
        self.release.return_value=False
        self.s._dispatch('switch','task_switch',self.data('B'))
        self.assertTrue(self.s.queue_blocked)
        self.assertEqual(self.s.active_task['tid'],'one')
        self.assertEqual([t['tid'] for t in self.s.waiting_tasks],['two'])
        self.tick('STATE_FINISH','A','t1')
        self.start.assert_called_once_with('A')

    def test_duplicate_switch_never_stops_twice(self):
        self.s._dispatch('switch','task_switch',self.data('A'))
        self.s._dispatch('switch','task_switch',self.data('A'))
        self.stop.assert_called_once()
        self.assertEqual(len(self.s.waiting_tasks),1)

    def test_switch_callback_and_wire_ack(self):
        msg={'tid':'s','method':'task_switch','data':self.data('A')}
        self.s.on_message(None,None,types.SimpleNamespace(retain=False,topic=m.SERVICE_TOPIC,payload=json.dumps(msg).encode()))
        self.s._dispatch(*self.s.control_queue.get_nowait())
        self.tick('STATE_CANCEL'); self.tick('STATE_DOING','A','t1')
        self.s.connected.set(); info=Mock(rc=0); info.is_published.return_value=True
        self.s.client.publish.return_value=info; self.s._flush_replies()
        sent=json.loads(self.s.client.publish.call_args.args[1])
        self.assertEqual(sent['method'],'task_switch')
        self.assertEqual(sent['data'],{'result':0})

if __name__ == '__main__':
    unittest.main()
