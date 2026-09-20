#!/usr/bin/env python3
"""接收中台quick_cruise，转为现有ROS inspection.start，等待真实启动回执。

只监听，不在启动时自动行驶。持久化tid去重，拒绝retained旧指令。
"""
import argparse
import json
import math
import os
import re
import sqlite3
import threading
import time
import uuid


def decode(raw):
    value = raw.decode('utf-8-sig') if isinstance(raw, bytes) else raw
    for _ in range(4):
        if isinstance(value, dict):
            return value
        if not isinstance(value, str):
            break
        try:
            value = json.loads(value)
        except ValueError:
            text = value.strip()
            if text.startswith('"{') and text.endswith('}"'):
                value = json.loads(text[1:-1])
            else:
                raise
    raise ValueError('指令必须为JSON对象')


def choose_task(tasks, explicit=''):
    if not isinstance(tasks, list):
        raise ValueError('机器人任务目录必须为列表')
    names = []
    for task in tasks:
        if not isinstance(task, (str, dict)):
            continue
        name = task if isinstance(task, str) else task.get('name', task.get('task_name', ''))
        if not isinstance(name, str):
            continue
        match = re.search(r'任务\s*0*(\d+)(?!\d)', str(name))
        if match and int(match.group(1)) == 0 and name not in names:
            names.append(name)
    if explicit:
        if explicit not in names:
            raise ValueError('指定任务不在当前地图任务0列表: ' + explicit)
        return explicit
    if len(names) != 1:
        raise ValueError('当前地图任务0数量为{}；请用--task-name指定完整任务名'.format(len(names)))
    return names[0]


def control_message(tid, name):
    return {'version': '1.0', 'message_id': uuid.uuid4().hex,
            'timestamp_ms': int(time.time() * 1000), 'type': 'inspection.control',
            'source': 'mqtt_quick_cruise_receiver', 'action': 'inspection.start',
            'tid': tid, 'command_id': tid, 'data': {'task_name': name, 'loop_time': 1}}


class Receiver:
    def __init__(self, args, ros, string_type, mqtt):
        self.args, self.ros, self.String = args, ros, string_type
        self.lock = threading.RLock()
        self.tasks, self.status = [], {}
        self.catalog_at = self.status_at = 0
        self.pending = None
        self.last_dispatch_at = 0
        self.command_topic = 'thing/robot/{}/services'.format(args.robot_code)
        self.reply_topic = 'thing/robot/{}/services_reply'.format(args.robot_code)
        os.makedirs(os.path.dirname(os.path.abspath(args.state_file)), exist_ok=True)
        import fcntl
        self.file_lock = open(args.state_file + '.lock', 'a')
        fcntl.flock(self.file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.db = sqlite3.connect(args.state_file, check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS commands (tid TEXT PRIMARY KEY, reply TEXT NOT NULL)')
        self.pub = ros.Publisher('/inspection/control', string_type, queue_size=10, latch=False)
        ros.Subscriber('/inspection/robot_catalog', string_type, self.catalog, queue_size=5)
        ros.Subscriber('/inspection/status', string_type, self.status_message, queue_size=5)
        ros.Subscriber('/inspection/task_status', string_type, self.task_reply, queue_size=20)
        options = {'client_id': args.robot_code + '-cruise-' + uuid.uuid4().hex[:10],
                   'protocol': mqtt.MQTTv311}
        if hasattr(mqtt, 'CallbackAPIVersion'):
            options['callback_api_version'] = mqtt.CallbackAPIVersion.VERSION2
        self.client = mqtt.Client(**options)
        self.client.username_pw_set(args.username, args.password)
        self.client.on_connect = self.connected
        self.client.on_subscribe = self.subscribed
        self.client.on_message = self.message
        self.client.on_connect_fail = lambda *a: ros.logerr('巡检MQTT连接失败，等待重连')
        self.client.connect_async(args.host, args.port, 30)
        self.client.loop_start()
        self.timer = ros.Timer(ros.Duration(1), self.check_timeout)
        ros.on_shutdown(self.shutdown)
        ros.loginfo('中台一键巡检接收程序已启动: topic=%s task=%s', self.command_topic,
                    args.task_name or '自动选择当前地图唯一任务0')

    def connected(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self.command_topic, qos=1)
        else:
            self.ros.logerr('巡检MQTT连接被拒绝: %s', rc)

    def subscribed(self, client, userdata, mid, codes, properties=None):
        failed = not codes or any(getattr(c, 'is_failure', False) or
                                  (isinstance(c, int) and c >= 128) for c in codes)
        if failed:
            self.ros.logerr('一键巡检主题订阅失败: %s', self.command_topic)
        else:
            self.ros.loginfo('已订阅中台巡检指令: %s', self.command_topic)

    def catalog(self, raw):
        try:
            msg = decode(raw.data)
            if msg.get('source') == 'robot_bridge':
                with self.lock:
                    self.tasks = msg.get('data', {}).get('tasks', [])
                    self.catalog_at = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.ros.logwarn('忽略无效机器人任务目录')

    def status_message(self, raw):
        try:
            msg = decode(raw.data)
            # 正式版及地图限定版继承 RealInspectionOrchestrator，消息来源
            # 与 ROS 节点名 inspection_orchestrator 不同。
            if msg.get('source') in ('inspection_orchestrator', 'real_inspection_orchestrator'):
                with self.lock:
                    self.status, self.status_at = msg, time.monotonic()
        except (ValueError, TypeError):
            pass

    def reply(self, tid, result, message):
        payload = {'tid': tid, 'method': 'quick_cruise', 'timestamp': int(time.time() * 1000),
                   'data': {'result': result, 'msg': message, 'robotCode': self.args.robot_code}}
        encoded = json.dumps(payload, ensure_ascii=False)
        self.db.execute('INSERT OR REPLACE INTO commands VALUES (?, ?)', (tid, encoded))
        self.db.commit()
        self.client.publish(self.reply_topic, encoded, qos=1, retain=False)
        self.ros.loginfo('一键巡检回执: tid=%s result=%s %s', tid, result, message)

    def message(self, client, userdata, raw):
        if raw.topic != self.command_topic or raw.retain:
            return
        try:
            msg = decode(raw.payload)
            if msg.get('method') != 'quick_cruise':
                return
            tid, data = msg.get('tid'), msg.get('data')
            if not isinstance(tid, str) or not tid.strip() or not isinstance(data, dict):
                raise ValueError('quick_cruise缺少tid或data')
            if data.get('robotCode') != self.args.robot_code:
                return
            with self.lock:
                cached = self.db.execute('SELECT reply FROM commands WHERE tid=?', (tid,)).fetchone()
                if cached:
                    if not self.pending or self.pending[0] != tid:
                        client.publish(self.reply_topic, cached[0], qos=1, retain=False)
                    return
                try:
                    stamp = msg.get('timestamp')
                    if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                            or abs(time.time() * 1000 - stamp) > 120000):
                        raise ValueError('指令时间戳缺失或超过120秒，请校准中台和机器人时间')
                    if self.pending:
                        raise ValueError('已有启动请求处理中')
                    now = time.monotonic()
                    missing = []
                    if not self.status_at:
                        missing.append('尚未收到巡检状态 /inspection/status，请检查巡检编排器及状态source')
                    elif now - self.status_at > 10:
                        missing.append('巡检状态超时 {:.1f}秒（上限10秒）'.format(now - self.status_at))
                    if not self.catalog_at:
                        missing.append('尚未收到真实任务目录 /inspection/robot_catalog，请检查robot_bridge及底盘连接')
                    elif now - self.catalog_at > 30:
                        missing.append('真实任务目录超时 {:.1f}秒（上限30秒）'.format(now - self.catalog_at))
                    if missing:
                        raise ValueError('；'.join(missing))
                    if self.status_at <= self.last_dispatch_at:
                        raise ValueError('等待下发后的新巡检状态，暂不接受新的启动请求')
                    if self.status.get('state') not in ('IDLE', 'FINISHED', 'STOPPED'):
                        raise ValueError('机器人巡检正在运行或尚未处于可启动状态')
                    robot = self.status.get('data', {}).get('robot_task', {})
                    if robot.get('task_state') in ('STATE_DOING', 'STATE_PAUSE'):
                        raise ValueError('底盘任务仍在执行或暂停中')
                    name = choose_task(self.tasks, self.args.task_name)
                    if not self.pub.get_num_connections():
                        raise ValueError('巡检编排器未订阅控制话题')
                except (ValueError, TypeError, AttributeError) as exc:
                    self.reply(tid, 1, str(exc))
                    return
                # 下发前持久化，崩溃后同tid不得再次驱动车体。
                unknown = {'tid': tid, 'method': 'quick_cruise', 'timestamp': int(time.time() * 1000),
                           'data': {'result': 1, 'robotCode': self.args.robot_code,
                                    'msg': '启动请求已登记，执行状态待核查；不重复下发'}}
                self.db.execute('INSERT INTO commands VALUES (?, ?)', (tid, json.dumps(unknown, ensure_ascii=False)))
                self.db.commit()
                self.pending = (tid, time.monotonic())
                self.last_dispatch_at = self.pending[1]
                self.pub.publish(self.String(data=json.dumps(control_message(tid, name), ensure_ascii=False)))
                self.ros.loginfo('中台一键巡检已转交编排器: task=%s tid=%s，等待真实启动回执', name, tid)
        except Exception as exc:
            self.ros.logerr('处理一键巡检指令失败: %s', exc)

    def task_reply(self, raw):
        try:
            msg = decode(raw.data)
            with self.lock:
                if not self.pending or msg.get('command_id') != self.pending[0]:
                    return
                state = msg.get('command_status')
                if state == 'success' and msg.get('source') == 'robot_bridge':
                    self.reply(self.pending[0], 0, '车体启动指令处理成功，不代表巡检已完成')
                elif state in ('failed', 'rejected', 'cancelled', 'canceled'):
                    self.reply(self.pending[0], 1, '启动失败: ' + str(msg.get('data', {})))
                else:
                    return
                self.pending = None
        except Exception as exc:
            self.ros.logerr('巡检启动回执处理失败: %s', exc)

    def check_timeout(self, event):
        with self.lock:
            if self.pending and time.monotonic() - self.pending[1] > 30:
                self.reply(self.pending[0], 1, '等待车体启动回执超时，执行状态未知；不会自动重发')
                self.pending = None

    def shutdown(self):
        self.timer.shutdown()
        self.client.disconnect()
        self.client.loop_stop()


def main():
    import rospy
    from std_msgs.msg import String
    import paho.mqtt.client as mqtt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-code', default='DT202600001')
    parser.add_argument('--host', default=os.environ.get('ROBOT_MQTT_HOST', '222.187.130.102'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('ROBOT_MQTT_PORT', '1883')))
    parser.add_argument('--username', default=os.environ.get('ROBOT_MQTT_USERNAME', 'autocar'))
    parser.add_argument('--password', default=os.environ.get('ROBOT_MQTT_PASSWORD', '123456'))
    parser.add_argument('--task-name', default='')
    parser.add_argument('--state-file', default=os.path.join(os.path.dirname(os.path.dirname(__file__)), 'quick_cruise_commands.sqlite3'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('mqtt_quick_cruise_receiver')
    Receiver(args, rospy, String, mqtt)
    rospy.spin()


if __name__ == '__main__':
    main()
