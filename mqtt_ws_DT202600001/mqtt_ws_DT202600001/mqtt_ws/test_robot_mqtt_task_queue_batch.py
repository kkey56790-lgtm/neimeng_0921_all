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
path = Path(__file__).with_name('robot_mqtt_all_in_one_task_queue_batch.py')
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




    def upload(self,names=('A','B'),tid='batch'):
        data={'robotCode':m.ROBOT_CODE,'tasks':[{'mainTaskName':n,'taskCode':m.generate_task_code(n)} for n in names]}
        self.s._dispatch(tid,'task_upload',data)

    def tick(self,state,name='',stamp='0'):
        self.s.next_task_poll=0
        self.status.return_value={'task_state':state,'main_task_name':name,'task_time':stamp,'queue_size':0,'remai_loop':0}
        self.s._advance_tasks()

    def msgs(self,method):return [e[0] for e in self.s.reply_outbox if e[0]['method']==method]

    def finish(self,name,stamp):
        self.tick('STATE_DOING',name,stamp)
        self.tick('STATE_FINISH',name,stamp+'-end')


    def state(self, state, name='A', queue=0, loop=0, point='point1'):
        self.s.next_task_poll=0
        self.status.return_value={'main_task_name':name,'task_state':state,'queue_size':queue,'remai_loop':loop,'task_time':point,'task_name':point}
        self.s._advance_tasks()

    def test_subpoint_finish_does_not_finish_main(self):
        self.upload();self.tick('STATE_CANCEL');self.state('STATE_DOING',queue=4)
        self.state('STATE_FINISH',queue=3,point='point2')
        self.state('STATE_FINISH',queue=3,point='point2')
        self.start.assert_called_once_with('A');self.assertFalse(self.msgs('task_finished'))
        self.state('STATE_DOING',queue=1,point='point3')
        self.state('STATE_FINISH',point='point4');self.assertFalse(self.msgs('task_finished'))
        self.state('STATE_FINISH',point='point4')
        self.assertEqual(self.msgs('task_finished')[-1]['data'],{'mainTaskName':'A','result':0})
        self.state('STATE_FINISH',point='point4')
        self.assertEqual([c.args[0] for c in self.start.call_args_list],['A','B'])
        self.state('STATE_DOING',name='B',queue=1)
        self.state('STATE_FINISH',name='B');self.state('STATE_FINISH',name='B')
        self.assertEqual(len(self.msgs('task_completed')),1)

    def test_stop_pause_fail_not_main_completion(self):
        self.upload();self.tick('STATE_CANCEL');self.state('STATE_DOING',queue=2)
        for state in ('STATE_CANCEL','STATE_STOPPED','STATE_PAUSE','STATE_FAIL'):
            self.state(state);self.state(state)
            self.assertIsNotNone(self.s.active_task)
        self.state('STATE_CANCEL',name='')
        self.start.assert_called_once_with('A');self.assertFalse(self.msgs('task_finished'))

    def test_loop_and_missing_fields_cannot_complete(self):
        self.upload();self.tick('STATE_CANCEL');self.state('STATE_DOING',queue=1)
        for loop in (-1,1,None):
            self.state('STATE_FINISH',loop=loop);self.state('STATE_FINISH',loop=loop)
        self.state('STATE_FINISH',queue=None);self.state('STATE_FINISH',queue=None)
        self.assertFalse(self.msgs('task_finished'))

    def test_confirmation_resets_on_new_point(self):
        self.upload();self.tick('STATE_CANCEL');self.state('STATE_DOING')
        self.state('STATE_FINISH');self.state('STATE_DOING',queue=2)
        self.state('STATE_FINISH');self.assertFalse(self.msgs('task_finished'))
        self.state('STATE_FINISH');self.assertEqual(len(self.msgs('task_finished')),1)

    def test_platform_stop_still_stops(self):
        self.upload();self.tick('STATE_CANCEL');self.state('STATE_DOING')
        self.s._dispatch('stop','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertIsNone(self.s.active_task);self.assertFalse(self.s.waiting_tasks)
        self.stop.assert_called_once()

    def test_point_telemetry_preserved(self):
        status={'main_task_name':'A','task_name':'point7','task_state':'STATE_DOING','queue_size':2,'remai_loop':0}
        wire=m.public_current_task(status)
        self.assertEqual(wire['task_name'],'point7')
        self.assertEqual(wire['main_task_name'],'A')

    def test_external_subtask_finish_not_permission_to_start(self):
        self.upload();self.state('STATE_FINISH',name='OTHER',queue=4)
        self.start.assert_not_called()

    def test_platform_station_codes_message(self):
        name='任务10'
        self.catalog.return_value=[{'name':name}]
        payload={'tid':'73dda467f4404950aef3b02b0c3e5477','method':'task_upload',
                 'timestamp':1789885408928,'data':{'robotCode':m.ROBOT_CODE,
                 'stationCode':['TASK-43EDCA6879D59AF7']}}
        msg=types.SimpleNamespace(retain=False,topic=m.SERVICE_TOPIC,payload=json.dumps(payload).encode())
        self.s.on_message(None,None,msg)
        self.s._dispatch(*self.s.control_queue.get_nowait())
        self.assertEqual(self.s.waiting_tasks[0]['mainTaskName'],name)
        self.assertEqual(self.s.waiting_tasks[0]['tid'],payload['tid'])
        self.s.on_message(None,None,msg)
        self.s._dispatch(*self.s.control_queue.get_nowait())
        self.assertEqual(len(self.s.waiting_tasks),1)

    def test_station_codes_preserve_order_and_atomic_validation(self):
        data={'robotCode':m.ROBOT_CODE,'stationCode':[m.generate_task_code('B'),m.generate_task_code('A')]}
        self.s._dispatch('codes','task_upload',data)
        self.assertEqual([t['mainTaskName'] for t in self.s.waiting_tasks],['B','A'])
        self.s._dispatch('bad','task_upload',dict(data,stationCode=[m.generate_task_code('A'),'UNKNOWN']))
        self.assertEqual(len(self.s.waiting_tasks),2)

    def test_invalid_station_code_shapes(self):
        for value in ([],None,'TASK-123',[None],[123],[[]]):
            with self.assertRaises(ValueError):
                m.parse_task_batch({'stationCode':value})

    def test_completed_main_cleared_in_reports_only(self):
        self.upload(('A',));self.tick('STATE_CANCEL')
        self.state('STATE_DOING');self.state('STATE_FINISH');self.state('STATE_FINISH')
        raw = dict(self.status.return_value)
        for method in ('osd', 'task_list'):
            message = {'method':method, 'data':{'currentTask':m.public_current_task(raw)}}
            self.s.publish_json(m.OSD_TOPIC,message)
            wire = json.loads(self.s.client.publish.call_args.args[1])
            current = wire['data']['currentTask']
            self.assertEqual(current['main_task_name'],'')
            self.assertEqual(current['taskCode'],'')
            self.assertEqual(current['task_name'],'point1')
            self.assertEqual(current['task_state'],'STATE_FINISH')
        self.assertEqual(raw['main_task_name'],'A')
        self.assertIsNone(self.s.active_task)
        self.assertEqual(len(self.msgs('task_finished')),1)
        # 同名重启、外部任务及未确认完成的点位不能被隐藏。
        for changes in ({'task_state':'STATE_DOING'}, {'task_time':'new-run'},
                        {'main_task_name':'B'}, {'queue_size':1}):
            status=dict(raw,**changes)
            message={'method':'osd','data':{'currentTask':m.public_current_task(status)}}
            self.s.publish_json(m.OSD_TOPIC,message)
            self.assertEqual(message['data']['currentTask']['main_task_name'],status['main_task_name'])

    def test_osd_log_uses_cleared_main_name(self):
        import contextlib
        import io
        self.upload(('A',));self.tick('STATE_CANCEL')
        self.state('STATE_DOING');self.state('STATE_FINISH');self.state('STATE_FINISH')
        self.s.connected.set()
        output=io.StringIO()
        with patch.object(m,'get_base_data',return_value={}), patch.object(m,'get_current_map',return_value={'name':'map'}), contextlib.redirect_stdout(output):
            self.s.publish_osd()
        self.assertIn('主任务= 当前任务=point1',output.getvalue())

if __name__=='__main__':unittest.main()
