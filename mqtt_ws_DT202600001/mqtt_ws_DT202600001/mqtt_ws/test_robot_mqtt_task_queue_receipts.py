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
path = Path(__file__).with_name('robot_mqtt_all_in_one_task_queue_receipts.py')
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



    def upload(self,name,tid):
        self.s._dispatch(tid,'task_upload',{'robotCode':m.ROBOT_CODE,'mainTaskName':name,'taskCode':m.generate_task_code(name)})

    def tick(self,state,name='',stamp='0'):
        self.s.next_task_poll=0
        self.status.return_value={'task_state':state,'main_task_name':name,'task_time':stamp}
        self.s._advance_tasks()

    def msgs(self,method):
        return [x[0] for x in self.s.reply_outbox if x[0]['method']==method]

    def test_two_stations_and_total(self):
        self.upload('A','one');self.upload('B','two');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1');self.tick('STATE_FINISH','A','2')
        self.assertEqual(len(self.msgs('task_finished')),1);self.assertFalse(self.msgs('task_completed'))
        self.tick('STATE_FINISH','A','2');self.tick('STATE_DOING','B','3');self.tick('STATE_FINISH','B','4')
        self.assertEqual([x['data'] for x in self.msgs('task_finished')],[{'mainTaskName':'A','result':0},{'mainTaskName':'B','result':0}])
        self.assertEqual(self.msgs('task_completed')[0]['data'],{'result':0})
        self.assertEqual(self.msgs('task_completed')[0]['tid'],'two')
        self.assertFalse(self.msgs('finished'))
        self.tick('STATE_FINISH','B','4');self.assertEqual(len(self.msgs('task_completed')),1)

    def test_wire_exact(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_FINISH','A','1')
        self.s.connected.set();info=Mock(rc=0);info.is_published.return_value=True
        self.s.client.publish.return_value=info;self.s._flush_replies()
        wire=[json.loads(c.args[1]) for c in self.s.client.publish.call_args_list]
        self.assertEqual([x['method'] for x in wire],['task_upload','task_finished','task_completed'])
        self.assertEqual(wire[1]['data'],{'mainTaskName':'A','result':0});self.assertEqual(wire[2]['data'],{'result':0})
        for x in wire:self.assertEqual(set(x),{'tid','method','timestamp','data'})

    def test_failure_not_completed(self):
        self.upload('A','one');self.upload('B','two');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1');self.tick('STATE_FAIL','A','2')
        self.assertFalse(self.msgs('task_completed'));self.assertFalse(self.msgs('task_finished'))

    def test_stop_not_completed(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1')
        self.s._dispatch('stop','task_stop',{'robotCode':m.ROBOT_CODE})
        self.assertFalse(self.msgs('task_completed'))

    def test_new_cycle_after_failure(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1');self.tick('STATE_FAIL','A','2')
        self.upload('B','two');self.tick('STATE_FAIL','A','2');self.tick('STATE_DOING','B','3');self.tick('STATE_FINISH','B','4')
        self.assertEqual(len(self.msgs('task_completed')),1)

    def test_unknown_not_completed(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1')
        self.s.active_task['last_valid']=-100;self.tick('')
        self.assertFalse(self.msgs('task_finished'));self.assertFalse(self.msgs('task_completed'))

    def test_duplicate_does_not_restart_cycle(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_FINISH','A','1')
        self.upload('A','one');self.assertEqual(len(self.msgs('task_completed')),1)
        self.assertEqual(self.s.command_history['one']['wire_reply']['method'],'task_finished')
        self.assertFalse(self.s.waiting_tasks)

    def test_switch_cancellation_not_full_success(self):
        self.upload('A','one');self.tick('STATE_CANCEL');self.tick('STATE_DOING','A','1')
        self.s._dispatch('switch','task_switch',{'robotCode':m.ROBOT_CODE,'mainTaskName':'B','taskCode':m.generate_task_code('B')})
        self.tick('STATE_CANCEL','A','1');self.tick('STATE_DOING','B','2');self.tick('STATE_FINISH','B','3')
        self.assertEqual(len(self.msgs('task_finished')),1);self.assertFalse(self.msgs('task_completed'))

if __name__ == '__main__':
    unittest.main()
