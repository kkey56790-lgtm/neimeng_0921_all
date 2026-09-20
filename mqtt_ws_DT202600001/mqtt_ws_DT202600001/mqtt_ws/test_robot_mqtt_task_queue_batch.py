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

if __name__=='__main__':unittest.main()
