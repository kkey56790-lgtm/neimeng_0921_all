"""jieguo 持久化任务队列。由 mqtt_yolo_point_uploader.py 启动。

检测项及计数以result中的name/content提交；taskName/taskId携带任务信息；图片通过签名PUT上传。
文件结果通知获得PUBACK后清理。协议未定义最终中台入库回执，不能据此
声称中台业务入库完成。上传累计失败三次后删除该任务目录；禁止同时运行两个本程序实例。
"""
import hashlib
import copy
import collections
import json
import mimetypes
import math
import os
import re
import shutil
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from urllib.parse import urlparse

import mqtt_yolo_point_uploader as base

_name_lock = threading.Lock()
_last_image_ms = 0


def resolve_task_code(task_name, supplied=None):
    """沿用robot_mqtt_all_in_one的taskCode生成规则，优先保留实际编码。"""
    if isinstance(supplied, str) and supplied.strip():
        return supplied.strip()
    return 'TASK-' + hashlib.sha256(str(task_name).encode('utf-8')).hexdigest().upper()[:16]


def task_image_name(task_name, directory, extension='.jpg'):
    """任务名_13位Unix毫秒时间戳；同毫秒调用顺延1ms避免重名。"""
    global _last_image_ms
    with _name_lock:
        stamp = max(base.now_ms(), _last_image_ms + 1)
        while True:
            name = '{}_{}{}'.format(base.safe_filename(task_name), stamp, extension.lower())
            if not os.path.exists(os.path.join(directory, name)):
                _last_image_ms = stamp
                return name
            stamp += 1


def read_json(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def batch_order(path):
    record = read_json(path)
    stamp = record.get('createdMs')
    if stamp is None:
        match = re.search(r'_(\d{13})$', os.path.basename(os.path.dirname(path)))
        stamp = int(match.group(1)) if match else int(os.path.getmtime(path) * 1000)
        record['createdMs'] = stamp
        base.atomic_write_json(path, record)
    return int(stamp), path


def task_directory_info(directory, results_root):
    if os.path.dirname(os.path.abspath(directory)) != os.path.abspath(results_root):
        return None
    match = re.fullmatch(r'(.+)_(\d{13})', os.path.basename(directory))
    if match:
        return match.group(1), int(match.group(2))
    return None


def reconcile_missing_files(record, path):
    """归档缺失引用，再移出待传列表；不把缺失文件记录为成功。"""
    if record.get('uploaded'):
        return  # 已成功上传、清理中断后的恢复仍使用原校验规则
    directory = os.path.dirname(path)
    missing = []
    for name in set(record.get('images', [])) | set(record.get('hashes', {})):
        if not name or os.path.basename(name) != name:
            raise ValueError('批次含非法文件名')
        target = os.path.join(directory, name)
        if os.path.islink(target):
            raise ValueError('批次文件是符号链接，拒绝处理')
        if not os.path.exists(target):
            missing.append(name)
    if not missing:
        return
    # 独立诊断记录包含原批次、计数、hash；不会被上传后清理。
    audit_path = os.path.join(directory, '.missing-' + os.path.basename(path)[len('.batch-'):])
    history = read_json(audit_path) if os.path.isfile(audit_path) else {'events': []}
    history['events'].append({'timeMs': base.now_ms(), 'missing': sorted(missing), 'record': copy.deepcopy(record)})
    base.atomic_write_json(audit_path, history)
    record['missingFiles'] = sorted(set(record.get('missingFiles', [])) | set(missing))
    record['images'] = [n for n in record.get('images', []) if n not in missing]
    for name in missing:
        record.setdefault('hashes', {}).pop(name, None)
    base.atomic_write_json(path, record)
    base.rospy.logwarn('批次缺失%d个文件，已记录到%s；继续处理存在的图片和文字，不视为上传成功', len(missing), audit_path)


def decode_mqtt_object(payload):
    """兼容标准对象、JSON字符串二次编码、裸对象外额外一对双引号。

    只解包外层，不替换正文反斜杠，也不改写签名URL或headers。
    """
    value = payload.decode('utf-8-sig') if isinstance(payload, bytes) else payload
    for _ in range(4):
        if isinstance(value, dict):
            return value
        if not isinstance(value, str):
            raise ValueError('MQTT消息根节点必须为JSON对象')
        value = value.strip()
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            if value.startswith('"{') and value.endswith('}"'):
                value = json.loads(value[1:-1])
            else:
                raise ValueError('回执不是有效JSON；请检查中台序列化格式') from None
    if isinstance(value, dict):
        return value
    raise ValueError('MQTT消息嵌套编码过多或根节点不是对象')


def report_text(task_name, items, counts):
    return "任务名：{}\n{}\n".format(task_name, "\n".join(
        "{}：{}".format(item["display_name"], counts.get(item["name"], 0))
        for item in items))


def compact_content(record, configured_items=None):
    """新批次使用结构化计数；旧批次提取检测行，去除任务/文件元数据。"""
    if record.get('items') and isinstance(record.get('counts'), dict):
        return ','.join('{}:{}'.format(item['display_name'],
                       int(record['counts'].get(item['name'], 0))) for item in record['items'])
    counts = {}

    def add(name, count):
        name = str(name).strip()
        if name and name not in ('任务名', '任务名称', 'taskName', 'pointName',
                                '视频总帧数', '实际检测帧数', '检出帧数', '检测项目种类总数'):
            counts[name] = max(counts.get(name, 0), int(count))

    def extract_object(value):
        if not isinstance(value, dict):
            return
        for item in value.get('detectionItems', []):
            if isinstance(item, dict) and str(item.get('count', '')).isdigit():
                add(item.get('name', ''), item['count'])
        for key in ('payload', 'data'):
            extract_object(value.get(key))
        content = value.get('content')
        if isinstance(content, str):
            extract_text(content)

    def extract_text(text):
        # 老版本的手动补传正文包含TXT文件名和完整result.json。
        decoder = json.JSONDecoder()
        for match in re.finditer(r'\{', text):
            try:
                obj, _ = decoder.raw_decode(text[match.start():])
                extract_object(obj)
                break
            except ValueError:
                continue
        for part in re.split(r'[\n\r,，;；]+', text):
            match = re.fullmatch(r'\s*([^:{}\[\]"\n]+?)\s*[:：]\s*(\d+)\s*', part)
            if match:
                add(match.group(1), match.group(2))

    extract_text(str(record.get('content', '')))
    items = record.get('items') or configured_items
    if items:
        # 只输出真实配置类别，兼容英文类别名和中文显示名，缺项补0。
        return ','.join('{}:{}'.format(item['display_name'], max(
            counts.get(item['name'], 0), counts.get(item['display_name'], 0))) for item in items)
    return ','.join('{}:{}'.format(name, count) for name, count in counts.items()) or '无检测结果'


def detected_content(record, configured_items=None):
    """仅发送已检出项目，文字缺失不阻止图片上传；不从图片臆造计数。"""
    result = []
    for part in compact_content(record, configured_items).split(','):
        name, separator, count = part.rpartition(':')
        if separator and count.isdigit() and int(count) > 0:
            result.append('{}:{}'.format(name, int(count)))
    return ','.join(result)


def prepare_put_headers(headers):
    """中文元数据按UTF-8字节发送，避免http.client默认Latin-1编码报错。

    不做URL编码、不删除签名头；最终签名是否匹配以对象存储响应为准。
    """
    if not isinstance(headers, dict):
        raise ValueError('凭证headers必须是对象')
    result = {}
    for name, value in headers.items():
        if not isinstance(name, str) or not isinstance(value, str):
            raise ValueError('凭证请求头名称和值必须为字符串')
        name.encode('ascii')
        if any(c in name + value for c in '\r\n\x00'):
            raise ValueError('凭证请求头含非法控制字符')
        result[name] = value if value.isascii() else value.encode('utf-8')
    return result


def image_upload_metadata(record, configured_items=None):
    """兼容未传完的旧记录，在发送时统一使用result/name/content。"""
    metadata = dict(record.get('imageMetadata', {}))
    old_items = metadata.pop('detectionItems', None)
    metadata.pop('content', None)
    if metadata.get('result') is None:
        if old_items is not None:
            metadata['result'] = [dict(name=item['name'], content=item['count'])
                                  for item in old_items]
        else:
            metadata['result'] = []
            for part in detected_content(record, configured_items).split(','):
                name, separator, count = part.rpartition(':')
                if separator and count.isdigit():
                    metadata['result'].append(dict(name=name, content=int(count)))
    return metadata


def normalize_result(result):
    if not isinstance(result, list):
        raise ValueError('result必须为识别结果数组')
    normalized = []
    for item in result:
        if not isinstance(item, dict) or 'name' not in item:
            raise ValueError('识别结果必须包含name')
        content = item.get('content', '')
        normalized.append({'name': str(item['name']),
                           'content': '' if content is None else str(content)})
    return normalized


def serialize_upload_message(payload):
    """请求中的result为识别数组，上传回执中的result为数字状态。"""
    payload = copy.deepcopy(payload)
    if payload.get('method') == 'file_upload_reply':
        data = payload['data']
        if not data.get('objectKey') or type(data.get('result')) is not int:
            raise ValueError('上传结果回执必须包含objectKey和整数result')
        return json.dumps(payload, ensure_ascii=False)
    if payload.get('method') in ('file_upload_request', 'file_upload_reply'):
        data = payload['data']
        result = data.get('result')
        if payload['method'] == 'file_upload_reply' and type(result) is int:
            result = None
        if result is None:
            result = data.get('detectionItems')
        if result is None:
            result = image_upload_metadata({'content': data.get('content') or ''})['result']
        normalized = normalize_result(result)
        if payload['method'] == 'file_upload_request':
            files = data.get('files', [])
            for file in files:
                # 单图旧请求可迁移元数据；多图不能把同一结果误配到所有图片。
                file['result'] = normalize_result(file.get('result', normalized if len(files) == 1 else []))
                file.setdefault('pointName', data.get('pointName', '') if len(files) == 1 else '')
            data.pop('result', None)
            data.pop('pointName', None)
        else:
            data['result'] = normalized
        data.pop('resultCode', None)
        data.pop('detectionItems', None)
        data.pop('content', None)
    return json.dumps(payload, ensure_ascii=False)


class FileOutbox(base.MqttOutbox):
    MAX_UPLOAD_FAILURES = 3

    def __init__(self, config, credentials, robot_code, dry_run=False):
        self.pending = {}
        self.pending_batches = {}
        self.credential_cache = []
        self.reply_condition = threading.Condition()
        self.response_timeout = float(config.get('upload_response_timeout', 120))
        if not math.isfinite(self.response_timeout) or self.response_timeout <= 0:
            raise ValueError('upload_response_timeout必须为正数')
        self.ready = threading.Event()
        config = copy.deepcopy(config)
        config['client_id'] = robot_code + '-jieguo-' + uuid.uuid4().hex[:10]
        base.rospy.loginfo('jieguo MQTT启动: host=%s port=%s client_id=%s dry_run=%s',
            config.get('broker', {}).get('host', base.DEFAULT_MQTT_HOST),
            config.get('broker', {}).get('port', base.DEFAULT_MQTT_PORT),
            config['client_id'], dry_run)
        super().__init__(config, credentials, robot_code, dry_run=dry_run)
        if self.client:
            self.client.on_message = self.on_message
            self.client.on_subscribe = self.on_subscribe

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        super()._on_connect(client, userdata, flags, rc, properties)
        self.ready.clear()
        if int(rc) == 0:
            result, mid = client.subscribe(self.topic, qos=1)
            if result != 0:
                base.rospy.logerr('订阅发送失败: topic=%s rc=%s，等待重连', self.topic, result)
            else:
                base.rospy.loginfo('已申请订阅凭证主题: %s mid=%s', self.topic, mid)

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self.ready.clear()
        super()._on_disconnect(client, userdata, rc, properties)

    def _on_connect_fail(self, client, userdata):
        base.rospy.logerr('MQTT网络连接失败，请检查地址、端口及网络；本地文件保留')

    def on_subscribe(self, client, userdata, mid, codes, properties=None):
        if codes and all(int(code) < 128 for code in codes):
            self.ready.set()
            base.rospy.loginfo('凭证主题订阅成功: %s，允许发送上传请求', self.topic)
            with self.condition:
                self.condition.notify_all()
        else:
            base.rospy.logerr('凭证主题订阅被拒绝: %s codes=%s，请检查MQTT账号订阅权限', self.topic, codes)

    def on_message(self, client, userdata, message):
        received_at = time.monotonic()
        if message.topic != self.topic or message.retain:
            return
        try:
            data = decode_mqtt_object(message.payload)
            if data.get("method") != "file_upload_response":
                return
            if not isinstance(data.get('data'), dict) or not isinstance(data.get('tid'), str):
                raise ValueError('凭证回执缺少合法的data对象或tid字符串')
            condition = getattr(self, 'reply_condition', self.condition)
            with condition:
                batches = getattr(self, 'pending_batches', {})
                target = next((key for key, expected in batches.items()
                               if key in self.pending and self.pending[key] is None
                               and self.credentials_match(data, *expected)), None)
                # 错误回执只能按原tid归属；不能把别批次失败应用到当前批次。
                if target is None and data['tid'] in self.pending:
                    if data['tid'] not in batches or data['data'].get('result') != 0:
                        target = data['tid']
                if target is not None:
                    self.pending[target] = data
                    condition.notify_all()
                    matched = True
                else:
                    matched = False
                    cache = getattr(self, 'credential_cache', [])
                    if data['data'].get('result') == 0 and data['data'].get('objectId'):
                        cache.append((time.monotonic(), data))
                        self.credential_cache = cache[-256:]
            # 日志不占用回执锁，避免阻塞正在等待的上传线程。
            base.rospy.loginfo('凭证回调: tid=%s matched=%s callback_monotonic=%.3f 解析及锁等待=%.3fs',
                               data.get('tid'), matched, received_at, time.monotonic() - received_at)
            if matched:
                base.rospy.loginfo('收到中台凭证回执: tid=%s result=%s',
                                  data.get('tid'), data.get('data', {}).get('result'))
            else:
                base.rospy.loginfo('凭证暂存，后续按objectId和文件名匹配: tid=%s', data.get('tid'))
        except (ValueError, AttributeError, TypeError) as exc:
            # 不打印原始凭证，避免泄漏签名URL。
            base.rospy.logwarn('MQTT回执解析失败: topic=%s bytes=%d reason=%s',
                               message.topic, len(message.payload), exc)

    def enqueue_path(self, path):
        with self.condition:
            if path not in self.items:
                self.items.append(path)
                self.items = collections.deque(sorted(self.items, key=batch_order))
            self.condition.notify_all()
        base.rospy.loginfo('已加入上传队列: %s', path)

    def publish_confirmed(self, payload):
        wire_message = serialize_upload_message(payload)
        base.rospy.loginfo('MQTT开始发送: topic=%s method=%s tid=%s',
                          self.topic, payload['method'], payload['tid'])
        if payload['method'] == 'file_upload_request':
            # 打印与publish完全相同的字符串，便于按tid核对中台接收结果。
            base.rospy.loginfo('MQTT上传请求原文[result-v2]: %s', wire_message)
        elif payload['method'] == 'file_upload_reply':
            base.rospy.loginfo('MQTT车体回执原文[result-v3]: %s', wire_message)
        info = self.client.publish(self.topic, wire_message,
                                   qos=1, retain=False)
        if info.rc != 0:
            raise RuntimeError('MQTT发布失败: rc={} method={}'.format(info.rc, payload['method']))
        info.wait_for_publish(timeout=15)
        if not info.is_published():
            raise RuntimeError('MQTT服务器确认超时: method=' + payload['method'])
        base.rospy.loginfo('MQTT服务器已确认收到: method=%s tid=%s mid=%s',
                          payload['method'], payload['tid'], info.mid)

    def send_file_receipt(self, record, path, name):
        """PUT成功后的通知单独落盘；重启只补通知，不再重复PUT。"""
        receipt = record['receipts'][name]
        if receipt.get('acknowledged'):
            if name not in record.setdefault('done', []):
                record['done'].append(name)
                base.atomic_write_json(path, record)
            return
        payload = {'tid': receipt['tid'], 'method': 'file_upload_reply',
                   'timestamp': base.now_ms(),
                   'data': {'objectKey': receipt['objectKey'], 'result': 0}}
        base.rospy.loginfo('发送图片上传成功回执: file=%s tid=%s objectKey=%s topic=%s',
                          name, receipt['tid'], receipt['objectKey'], getattr(self, 'topic', 'file'))
        self.publish_confirmed(payload)
        receipt['acknowledged'] = True
        receipt['acknowledgedMs'] = base.now_ms()
        if name not in record.setdefault('done', []):
            record['done'].append(name)
        base.atomic_write_json(path, record)
        base.rospy.loginfo('图片成功回执已获MQTT确认: %s', name)

    def send_file_failure(self, tid, object_key):
        """失败通知不标记图片完成，也不改变原上传重试流程。"""
        try:
            self.publish_confirmed({'tid': tid, 'method': 'file_upload_reply',
                                    'timestamp': base.now_ms(),
                                    'data': {'objectKey': object_key, 'result': -1}})
        except Exception:
            base.rospy.logwarn('图片失败回执未获确认，将随图片后续重试继续处理: tid=%s', tid)

    @staticmethod
    def credentials_match(reply, object_id, names):
        data = reply.get('data', {})
        if type(data.get('result')) is not int or data['result'] != 0 or data.get('objectId') != object_id:
            return False
        expiry = data.get('expiredTime')
        if expiry is not None:
            try:
                if float(expiry) <= base.now_ms() + 1000:
                    return False
            except (ValueError, TypeError):
                return False
        urls = data.get('urls', [])
        if not isinstance(urls, list):
            return False
        return any(isinstance(u, dict) and u.get('fileName') in names
                   and u.get('objectKey') and isinstance(u.get('putUrl'), str)
                   and urlparse(u['putUrl']).scheme in ('http', 'https') for u in urls)

    def request(self, record, files):
        if not files:
            raise ValueError('禁止发送files为空的上传请求')
        record['taskCode'] = resolve_task_code(record['taskName'], record.get('taskCode'))
        tid = uuid.uuid4().hex
        payload = {"tid": tid, "method": "file_upload_request", "timestamp": base.now_ms(),
                   "data": {"robotCode": self.robot_code,
                            "stationCode": record["taskCode"],
                            "stationName": record["taskName"], "objectId": record["objectId"],
                            "taskName": record["taskName"], "taskId": record["taskId"],
                            "expireMinutes": 30, "files": files}}
        # 每张图片独立批次，重试仍使用截图时固化的元数据。
        metadata = image_upload_metadata(record, getattr(self, 'detection_items', None))
        result = normalize_result(metadata.pop('result'))
        point_name = metadata.pop('pointName', '')
        payload['data'].update(metadata)
        payload['data']['files'] = copy.deepcopy(files)
        for file in payload['data']['files']:
            file['result'] = normalize_result(file.get('result', result if len(record.get('images', [])) == 1 else []))
            file.setdefault('pointName', point_name if len(record.get('images', [])) == 1 else '')
        condition = getattr(self, 'reply_condition', self.condition)
        with condition:
            expected_names = {f['fileName'] for f in files}
            self.credential_cache = [(stamp, reply) for stamp, reply in getattr(self, 'credential_cache', [])
                                     if time.monotonic() - stamp < 1800]
            for index, (_, reply) in enumerate(self.credential_cache):
                if self.credentials_match(reply, record['objectId'], expected_names):
                    self.credential_cache.pop(index)
                    base.rospy.loginfo('使用迟到凭证: objectId=%s tid=%s', record['objectId'], reply['tid'])
                    return reply['tid'], reply['data']
            if not hasattr(self, 'pending_batches'):
                self.pending_batches = {}
            self.pending_batches[tid] = (record['objectId'], expected_names)
            self.pending[tid] = None
        summary = str([f['result'] for f in payload['data']['files']]).replace('\n', ' ').replace('\r', ' ')
        if len(summary) > 100:
            summary = summary[:100] + '…'
        base.rospy.loginfo('上传请求: task=%s 图片=%d 检测=%s',
                           str(record['taskName'])[:60], len(files), summary or '无')
        try:
            self.publish_confirmed(payload)
            timeout = getattr(self, 'response_timeout', 120)
            waiting_since = time.monotonic()
            base.rospy.loginfo('等待中台凭证: tid=%s timeout=%.1fs', tid, timeout)
            deadline = waiting_since + timeout
            with condition:
                while self.pending[tid] is None and not self.stopping:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RuntimeError('上传凭证回执超时: tid={} waited={:.1f}s'.format(tid, timeout))
                    condition.wait(min(remaining, 1))
                if self.stopping:
                    raise RuntimeError("程序正在退出")
                reply = self.pending[tid]
                data = reply['data']
            base.rospy.loginfo('凭证等待结束: tid=%s elapsed=%.3fs', tid, time.monotonic() - waiting_since)
            if type(data.get("result")) is not int or data["result"] != 0:
                raise RuntimeError("中台拒绝文件上传请求")
            if data.get("objectId", record["objectId"]) != record["objectId"]:
                raise RuntimeError("上传凭证objectId不匹配")
            return reply['tid'], data
        finally:
            with condition:
                self.pending.pop(tid, None)
                self.pending_batches.pop(tid, None)

    def upload(self, path):
        record = read_json(path)
        if record.get('state') == 'MISSING_FILES':
            return
        if record.get('state') == 'COLLECTING':
            base.rospy.loginfo('任务尚未结束，保留采集批次，暂不上传: %s', path)
            return
        reconcile_missing_files(record, path)
        if (not record.get('images') and not record.get('hashes')
                and not record.get('counts')
                and record.get('content', '') in ('', '启动补传图片（无TXT）', '无检测结果')
                and record.get('missingFiles')):
            record['state'] = 'MISSING_FILES'
            base.atomic_write_json(path, record)
            base.rospy.logwarn('批次文件已全部缺失且无检测文字，保留记录并停止重试: %s', path)
            return
        root = os.path.dirname(path)
        if not record.get('uploaded') and not record.get('images'):
            base.rospy.logwarn('批次没有可上传图片，不发送请求，保留本地结果: %s', path)
            return
        # 仅允许清理本批记录引用的jieguo直属普通文件，拒绝路径穿越及链接。
        def local(name):
            if not name or os.path.basename(name) != name:
                raise ValueError("非法文件名")
            target = os.path.join(root, name)
            if os.path.islink(target):
                raise ValueError("拒绝读取或删除符号链接")
            return target

        for name, digest in record["hashes"].items():
            target = local(name)
            if not os.path.exists(target) and record.get("uploaded"):
                continue  # 清理途中重启
            with open(target, "rb") as stream:
                if hashlib.sha256(stream.read()).hexdigest() != digest:
                    raise RuntimeError("文件已被修改，保留现场: " + name)
        if not record.get("uploaded"):
            for name in record.get('receipts', {}):
                self.send_file_receipt(record, path, name)
            remaining = [n for n in record['images'] if n not in record.get('done', [])]
            files = []
            for name in remaining:
                target = local(name)
                with open(target, "rb") as stream:
                    blob = stream.read()
                # 旧图保留本地路径，持久化上传文件名，避免重试时改名或丢失清理引用。
                names = record.setdefault('uploadNames', {})
                if name not in names:
                    pattern = re.escape(base.safe_filename(record['taskName'])) + r'_\d{13}\.[A-Za-z0-9]+'
                    names[name] = name if re.fullmatch(pattern, name) else task_image_name(
                        record['taskName'], root, os.path.splitext(name)[1])
                    base.atomic_write_json(path, record)
                upload_name = names[name]
                mime = mimetypes.guess_type(name)[0] or "application/octet-stream"
                files.append({"fileName": upload_name, "fileSize": len(blob),
                    "mimeType": mime, "mediaType": 1, "md5": hashlib.md5(blob).hexdigest()})
            if files:
                base.rospy.loginfo('任务统一申请上传: task=%s 图片数=%d', record['taskName'], len(files))
                tid, response = self.request(record, files)
            for name in remaining:
                upload_name = record['uploadNames'][name]
                with open(local(name), 'rb') as stream:
                    blob = stream.read()
                if hashlib.sha256(blob).hexdigest() != record['hashes'][name]:
                    raise RuntimeError('申请凭证期间图片被修改，停止上传: ' + name)
                urls = [u for u in response.get("urls", []) if u.get("fileName") == upload_name]
                if not urls:
                    base.rospy.loginfo('本次凭证未包含图片，留待下一次申请: %s', upload_name)
                    continue
                if len(urls) != 1 or not urls[0].get("objectKey"):
                    raise RuntimeError("上传凭证缺少对应文件或objectKey")
                entry = urls[0]
                try:
                    import requests
                except ImportError as exc:
                    raise RuntimeError('已获得中台凭证，但缺少HTTP上传依赖；请运行 python3 -m pip install requests；文件保留') from exc
                if urlparse(entry.get("putUrl", "")).scheme not in ("http", "https"):
                    raise RuntimeError("非法PUT地址")
                headers = prepare_put_headers(entry.get('headers', {}))
                endpoint = urlparse(entry['putUrl'])
                base.rospy.loginfo('开始HTTP PUT上传: file=%s bytes=%d server=%s；连接超时10秒，响应超时30秒',
                                   name, len(blob), endpoint.netloc)
                # 禁止重定向；不携带MQTT账号，只使用协议返回的签名头。
                started = time.monotonic()
                try:
                    result = requests.put(entry["putUrl"], data=blob, headers=headers,
                                          timeout=(10, 30), allow_redirects=False)
                except Exception as exc:
                    # 不打印异常正文，requests网络异常可能包含完整签名URL。
                    self.send_file_failure(tid, entry['objectKey'])
                    raise RuntimeError('HTTP PUT未完成: {} server={} elapsed={:.1f}s；请检查上传端口及请求头，文件保留'.format(
                        type(exc).__name__, endpoint.netloc, time.monotonic() - started)) from None
                base.rospy.loginfo('HTTP PUT响应: file=%s status=%s elapsed=%.1fs',
                                   name, result.status_code, time.monotonic() - started)
                if not 200 <= result.status_code < 300:
                    self.send_file_failure(tid, entry['objectKey'])
                    code = ''
                    try:
                        code = ET.fromstring(result.text).findtext('Code', '')[:120]
                    except (ET.ParseError, TypeError):
                        pass
                    raise RuntimeError('HTTP PUT失败: status={} storageCode={}；文件保留'.format(result.status_code, code))
                record.setdefault('receipts', {})[name] = {
                    'tid': tid, 'objectKey': entry['objectKey'], 'putCompletedMs': base.now_ms(),
                    'acknowledged': False}
                base.atomic_write_json(path, record)
                self.send_file_receipt(record, path, name)
            if any(n not in record.get('done', []) for n in record['images']):
                raise RuntimeError('部分图片已上传，其余图片等待凭证；保留整批文件并继续申请')
            record["uploaded"] = True
            base.atomic_write_json(path, record)
        # 在任务目录之外留存回执证据，不保留签名URL或凭证headers。
        if record.get('managedDirectory'):
            results_root = os.path.realpath(getattr(self, 'results_root', ''))
            if (os.path.islink(root) or os.path.dirname(os.path.realpath(root)) != results_root
                    or not re.fullmatch(re.escape(base.safe_filename(record['taskName'])) + r'_\d{13}', os.path.basename(root))):
                raise RuntimeError('任务目录不在指定jieguo直属路径内，拒绝清理')
            audit_dir = os.path.join(results_root, '.upload_receipts')
            if os.path.islink(audit_dir):
                raise RuntimeError('回执目录不允许符号链接')
            base.atomic_write_json(os.path.join(audit_dir, base.safe_filename(record['objectId']) + '.json'), {
                'taskName': record['taskName'], 'objectId': record['objectId'],
                'completedMs': base.now_ms(), 'confirmation': 'HTTP PUT success + MQTT PUBACK; not platform database ack',
                'receipts': record.get('receipts', {}), 'missingFiles': record.get('missingFiles', [])})
        for name in record["hashes"]:
            target = local(name)
            if os.path.exists(target):
                with open(target, "rb") as stream:
                    if hashlib.sha256(stream.read()).hexdigest() != record["hashes"][name]:
                        raise RuntimeError("上传期间文件被修改，拒绝删除: " + name)
                os.remove(target)
        os.remove(path)
        base.rospy.loginfo("图片PUT及结果通知完成，已清理批次: %s", record["taskName"])
        if record.get('managedDirectory'):
            if not os.listdir(root):
                os.rmdir(root)
                base.rospy.loginfo('任务上传及回执完成，已删除任务文件夹: %s', root)
            else:
                base.rospy.logwarn('任务目录仍有未归档文件或缺失记录，保留目录，不递归删除: %s', root)

    def _delete_failed_task(self, path, record):
        """只删除已封存的独立任务目录；绝不递归删除结果根目录。"""
        if record.get('singleImage'):
            root = os.path.realpath(getattr(self, 'results_root', ''))
            if (os.path.islink(path) or os.path.realpath(os.path.dirname(path)) != root
                    or record.get('state') != 'READY'):
                raise RuntimeError('单图批次不在结果根目录，拒绝清理')
            targets = []
            for name, digest in record['hashes'].items():
                target = os.path.join(root, name)
                if (os.path.basename(name) != name or os.path.islink(target)
                        or os.path.dirname(os.path.realpath(target)) != root):
                    raise RuntimeError('单图批次文件路径非法')
                if os.path.exists(target):
                    with open(target, 'rb') as stream:
                        if hashlib.sha256(stream.read()).hexdigest() != digest:
                            raise RuntimeError('单图批次文件被修改，拒绝清理')
                    targets.append(target)
            for target in targets:
                os.remove(target)
            os.remove(path)
            base.rospy.logwarn('单图上传累计失败三次，已清理本图片及附带信息: %s', path)
            return
        configured_root = getattr(self, 'results_root', '')
        root = os.path.realpath(configured_root)
        directory = os.path.dirname(os.path.abspath(path))
        resolved = os.path.realpath(directory)
        expected = re.escape(base.safe_filename(record['taskName'])) + r'_\d{13}'
        if (not configured_root or not record.get('managedDirectory')
                or os.path.islink(directory) or os.path.islink(path)
                or os.path.dirname(resolved) != root or resolved == root
                or not re.fullmatch(expected, os.path.basename(directory))
                or record.get('state') == 'COLLECTING'):
            raise RuntimeError('失败批次不是jieguo直属已封存任务目录，拒绝自动删除: ' + path)
        # 先检查整棵目录，避免越过链接/Windows junction或删除正在续采的批次。
        for current, dirs, files in os.walk(directory, followlinks=False):
            for name in dirs + files:
                target = os.path.join(current, name)
                real = os.path.realpath(target)
                if (os.path.islink(target)
                        or getattr(os.path, 'isjunction', lambda _: False)(target)
                        or os.path.commonpath([resolved, real]) != resolved):
                    raise RuntimeError('任务目录包含链接或越界路径，拒绝自动删除')
                if name.startswith('.batch-') and name.endswith('.json'):
                    if read_json(target).get('state') == 'COLLECTING':
                        raise RuntimeError('任务目录仍在采集，拒绝自动删除')
        shutil.rmtree(directory)
        with self.condition:
            self.items = collections.deque(
                item for item in self.items
                if os.path.dirname(os.path.abspath(item)) != directory)
        base.rospy.logwarn('上传累计失败三次，已删除任务目录及全部图片/文字/批次记录: task=%s directory=%s',
                           record['taskName'], directory)

    def upload_with_retry_limit(self, path):
        record = read_json(path)
        if int(record.get('uploadFailures', 0)) >= self.MAX_UPLOAD_FAILURES:
            self._delete_failed_task(path, record)
            return
        try:
            self.upload(path)
        except Exception:
            if self.stopping or not os.path.isfile(path):
                raise  # 主动退出不算一次上传失败。
            record = read_json(path)  # 保留本轮已成功PUT/通知的进度。
            if record.get('uploaded') or record.get('state') == 'COLLECTING':
                raise  # 成功后的清理错误不算上传失败。
            record['uploadFailures'] = int(record.get('uploadFailures', 0)) + 1
            record['lastUploadFailureMs'] = base.now_ms()
            base.atomic_write_json(path, record)
            base.rospy.logwarn('任务上传失败: task=%s 次数=%d/%d',
                               record['taskName'], record['uploadFailures'], self.MAX_UPLOAD_FAILURES)
            if record['uploadFailures'] >= self.MAX_UPLOAD_FAILURES:
                self._delete_failed_task(path, record)
                return
            raise

    def run(self):
        while not self.stopping:
            with self.condition:
                if not self.items or (not self.dry_run and not self.ready.is_set()):
                    if self.items:
                        base.rospy.logwarn_throttle(10, '待上传批次=%d，等待MQTT连接/凭证主题订阅成功', len(self.items))
                    self.condition.wait(1)
                    continue
                path = self.items.popleft()
            try:
                if self.dry_run:
                    print("DRY_RUN 保留文件: " + path, flush=True)
                else:
                    base.rospy.loginfo('开始处理待上传批次: %s', path)
                    self.upload_with_retry_limit(path)
            except Exception as exc:
                base.rospy.logwarn("上传或清理未完成，将重试（上传最多失败三次）: %s", exc)
                with self.condition:
                    self.items.appendleft(path)
                    self.condition.wait(5)


class JieguoUploader(base.YoloPointMqttUploader):
    outbox_class = FileOutbox

    def __init__(self, args):
        self.snapshot_interval = max(0.2, args.snapshot_interval)
        self.record = None
        self.direct_task_status_seen = False
        self.last_robot_status_ms = 0
        self.collection_paused = False
        self.recovered_collecting = {}
        # 跨进程锁避免同名节点启动前的补传线程同时上传/删除同一批文件。
        import fcntl
        os.makedirs(args.results_root, exist_ok=True)
        self.process_lock = open(os.path.join(args.results_root, ".uploader.lock"), "a")
        fcntl.flock(self.process_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        super().__init__(args)
        self.robot_task_subscription = base.rospy.Subscriber(
            '/inspection/task_status', base.String, self._robot_task_callback, queue_size=50)
        base.rospy.loginfo('已监听底盘真实任务状态: /inspection/task_status')

    def _save_record(self):
        if self.record.get('state') == 'COLLECTING' and self.record.get('txt'):
            base.atomic_write_bytes(os.path.join(os.path.dirname(self.record_path), self.record['txt']),
                report_text(self.record['taskName'], self.record.get('items', self.items),
                            self.record['counts']).encode('utf-8'))
        base.atomic_write_json(self.record_path, self.record)

    def _open_task_directory(self):
        # 进程重启不是任务结束；等待真实状态后接着写原任务目录。
        for path, record in list(self.recovered_collecting.items()):
            if record['taskName'] != self.current_task_name:
                continue
            self.record_path, self.record = path, record
            self.task_dir = os.path.dirname(path)
            self.current_task_id = record['taskId']
            self.current_task_identity = record['taskId'] or record['taskName']
            self.active_collection = base.PointCollection(
                self.current_task_id, self.current_task_name, '任务全程', self.items, 1)
            self.active_collection.counts.update(record['counts'])
            self.last_snapshot = 0
            del self.recovered_collecting[path]
            base.rospy.loginfo('继续采集未结束任务: %s', self.task_dir)
            return
        directory_name = task_image_name(self.current_task_name, self.results_root, extension='')
        self.task_dir = os.path.join(self.results_root, directory_name)
        os.mkdir(self.task_dir)
        self.record_path = os.path.join(self.task_dir, ".batch-" + uuid.uuid4().hex + ".json")
        self.record = {"taskId": self.current_task_id, "taskName": self.current_task_name,
                       "objectId": self.robot_code + "-" + uuid.uuid4().hex,
                       "createdMs": int(directory_name.rsplit('_', 1)[1]), "managedDirectory": True,
                       "state": "COLLECTING", "images": [], "hashes": {},
                       "txt": base.safe_filename(self.current_task_name) + '.txt',
                       "counts": {i["name"]: 0 for i in self.items}, "items": self.items, "content": ""}
        self.active_collection = base.PointCollection(self.current_task_id, self.current_task_name,
                                                      "任务全程", self.items, 1)
        self.last_snapshot = 0
        self._save_record()
        base.rospy.loginfo('当前任务独立截图目录: %s', self.task_dir)

    def _point_callback(self, raw):
        try:
            message = base.json_load(raw)
            with self.lock:
                if self.active_collection and message.get("event") == "arrived":
                    self.active_collection.point_name = str(message.get("point_name", ""))
        except Exception:
            pass

    def _apply_robot_task(self, robot):
        state = str(robot.get('task_state', '')).upper()
        name = str(robot.get('main_task_name') or '').strip()
        if state in ('STATE_DOING', 'STATE_PAUSE', 'STATE_PAUSED'):
            if not name:
                return False
            # task_name/task_type/task_time描述当前子任务，不能用来切分主任务。
            if self.task_active and name == self.current_task_name:
                identity = self.current_task_id
            else:
                identity = uuid.uuid4().hex
            self._begin_task(identity, name, 'RUNNING')
            # 暂停/等待仍属于本次巡检，继续识别和保存，不提前上传。
            self.collection_paused = False
            return True
        if state in ('STATE_FINISH', 'STATE_FINISHED', 'STATE_IDLE', 'STATE_STOP',
                     'STATE_STOPPED', 'STATE_CANCEL', 'STATE_CANCELED',
                     'STATE_CANCELLED', 'STATE_FAIL', 'STATE_FAILED'):
            if state in ('STATE_FINISH', 'STATE_FINISHED', 'STATE_IDLE'):
                # 子任务完成但队列尚有到点/暂停任务时，继续当前归档。
                try:
                    if int(robot.get('queue_size', 0)) > 0:
                        return True
                except (TypeError, ValueError):
                    return False
            for path, record in list(self.recovered_collecting.items()):
                if not name or name == record['taskName']:
                    record['finishReason'] = state
                    record['finishedMs'] = base.now_ms()
                    self.finalize(record, path)
                    del self.recovered_collecting[path]
            if self.task_active and (not name or name == self.current_task_name):
                self._begin_task(self.current_task_id, self.current_task_name, 'FINISHED')
            self.collection_paused = False
            return True
        return False

    def _robot_task_callback(self, raw):
        try:
            message = base.json_load(raw)
            if message.get('source') != 'robot_bridge':
                return
            robot = message.get('data', {})
            if not isinstance(robot, dict) or not robot.get('task_state'):
                return  # 启动命令的HTTP回执不等同于周期任务状态
            with self.lock:
                stamp = int(message.get('timestamp_ms') or 0)
                if stamp and stamp < self.last_robot_status_ms:
                    return
                if self._apply_robot_task(robot):
                    self.direct_task_status_seen = True
                    self.last_robot_status_ms = stamp or self.last_robot_status_ms
        except Exception as exc:
            base.rospy.logwarn_throttle(5, '底盘任务状态解析失败: %s', exc)

    def _status_callback(self, raw):
        try:
            message = base.json_load(raw)
            robot = message.get("data", {}).get("robot_task", {})
            with self.lock:
                if self.direct_task_status_seen:
                    return  # 避免编排器旧快照把任务切回上一任务
                if isinstance(robot, dict) and robot.get('task_state'):
                    self._apply_robot_task(robot)
                else:
                    super()._status_callback(raw)
        except Exception as exc:
            base.rospy.logwarn_throttle(5, "任务状态解析失败: %s", exc)

    def _collect_frame(self, frame):
        with self.lock:
            if getattr(self, 'collection_paused', False):
                return
            # 只以本次检测对应的画面决定是否截图，不沿用之前的目标框。
            if self.active_collection is not None:
                self.active_collection.best_image = None
                self.active_collection.best_objects = []
                self.active_collection.best_score = 0
            super()._collect_frame(frame)
            collection = self.active_collection
            if collection is None or self.record is None:
                return
            if collection.counts != self.record["counts"]:
                self.record["counts"] = dict(collection.counts)
                self._save_record()
            if time.monotonic() - self.last_snapshot < self.snapshot_interval:
                return
            if collection.best_image is None or not collection.best_objects:
                return
            stem = os.path.splitext(task_image_name(self.current_task_name, self.task_dir))[0]
            image = self._save_screenshot(collection, stem)
            if image:
                self.record["images"].append(os.path.basename(image))
                self._save_record()
                self.last_snapshot = time.monotonic()
                base.rospy.loginfo("截图已保存: %s", image)
            collection.best_image = None
            collection.best_score = 0

    def _save_screenshot(self, collection, stem):
        if base.cv2 is None or collection.best_image is None or not self.screenshots_enabled:
            return ""
        height, width = collection.best_image.shape[:2]
        valid = []
        for obj in collection.best_objects:
            bbox = obj.get('bbox', [])
            if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
                continue
            try:
                coords = [float(v) for v in bbox[:4]]
                if not all(math.isfinite(v) for v in coords):
                    continue
                x1, y1, x2, y2 = [int(v) for v in coords]
                x1, x2 = sorted(max(0, min(width - 1, v)) for v in (x1, x2))
                y1, y2 = sorted(max(0, min(height - 1, v)) for v in (y1, y2))
                if x2 > x1 and y2 > y1:
                    valid.append(obj)
            except (TypeError, ValueError, OverflowError):
                continue
        if not valid:
            return ""
        selected = copy.copy(collection)
        selected.best_objects = valid
        return super()._save_screenshot(selected, stem)

    def finalize(self, record, path):
        reconcile_missing_files(record, path)
        directory = os.path.dirname(path)
        name = record.get('txt') or base.safe_filename(record["taskName"]) + ".txt"
        if os.path.basename(name) != name:
            raise ValueError('非法TXT文件名')
        if not record.get('txt') and os.path.exists(os.path.join(directory, name)):
            name = base.safe_filename(record["taskName"]) + "_" + uuid.uuid4().hex[:10] + ".txt"
        record["content"] = report_text(record["taskName"], record.get("items", self.items), record["counts"])
        base.atomic_write_bytes(os.path.join(directory, name), record["content"].encode("utf-8"))
        record["txt"] = name
        record["state"] = "READY"
        for filename in record["images"] + [name]:
            with open(os.path.join(directory, filename), "rb") as stream:
                record["hashes"][filename] = hashlib.sha256(stream.read()).hexdigest()
        base.atomic_write_json(path, record)
        self.outbox.enqueue_path(path)

    def _close_current_task_locked(self, reason):
        if self.record is not None:
            self.record['finishReason'] = reason
            self.record['finishedMs'] = base.now_ms()
            self.finalize(self.record, self.record_path)
            base.rospy.loginfo('任务切换或结束，结果已封存入队: task=%s reason=%s 图片=%d',
                               self.record['taskName'], reason, len(self.record['images']))
        self.record = None
        self.task_dir = ""
        self.active_collection = None

    def _recover_ready_tasks(self):
        self.outbox.detection_items = self.items
        self.outbox.results_root = self.results_root
        base.rospy.loginfo('启动扫描结果目录: %s', self.results_root)
        for directory, dirs, _ in os.walk(self.results_root, followlinks=False):
            dirs[:] = sorted(d for d in dirs if d != '.upload_receipts' and not os.path.islink(os.path.join(directory, d)))
            try:
                self._recover_directory(directory)
            except Exception as exc:
                base.rospy.logerr('启动补传扫描失败，保留该目录文件并继续扫描其他目录: %s: %s', directory, exc)

    def _recover_directory(self, directory):
        tracked = set()
        invalid_batch = False
        directory_info = task_directory_info(directory, self.results_root)
        for name in sorted(os.listdir(directory)):
            if not name.startswith(".batch-") or not name.endswith(".json"):
                continue
            path = os.path.join(directory, name)
            try:
                if os.path.islink(path):
                    raise ValueError('拒绝读取符号链接批次: ' + path)
                record = read_json(path)
                if directory_info:
                    task_name, created_ms = directory_info
                    # 目录名经过字符替换和截断；上传必须保留原始任务名称。
                    task_name = record.get('taskName') or task_name
                    record.update(taskName=task_name, createdMs=created_ms, managedDirectory=True)
                    base.atomic_write_json(path, record)
                tracked.update(record['images'])
                tracked.update(record['hashes'])
                if record.get('txt'):
                    tracked.add(record['txt'])
                if record['state'] == 'MISSING_FILES':
                    continue
                if record["state"] == "COLLECTING":
                    self.recovered_collecting[path] = record
                    base.rospy.loginfo('未结束批次等待底盘状态，不提前上传: task=%s 图片=%d',
                                       record['taskName'], len(record['images']))
                else:
                    self.outbox.enqueue_path(path)
            except Exception as exc:
                invalid_batch = True
                base.rospy.logerr('跳过异常批次，继续其他批次: %s: %s', path, exc)
        if invalid_batch:
            # 无法读出损坏记录的文件归属时，不能把这些文件误当新批次重复上传。
            base.rospy.logwarn('目录含异常批次，暂缓该目录未归档文件补传；已读取的正常批次继续上传: %s', directory)
            return
        # 手工放入目录的图片/TXT作为一批补传，TXT内容原样合并，不猜图片归属。
        image_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff')
        loose = [n for n in sorted(os.listdir(directory))
                 if n not in tracked and n.lower().endswith(image_extensions + ('.txt', '.result.json'))
                 and os.path.isfile(os.path.join(directory, n))
                 and not os.path.islink(os.path.join(directory, n))]
        if loose:
            texts = []
            for name in loose:
                if name.lower().endswith(('.txt', '.result.json')):
                    with open(os.path.join(directory, name), 'rb') as stream:
                        raw = stream.read()
                    try:
                        content = raw.decode('utf-8-sig')
                    except UnicodeDecodeError:
                        content = raw.decode('gb18030')
                    texts.append(name + "\n" + content)
            task_name = os.path.basename(directory) if directory != self.results_root else '启动补传'
            txt_names = [n for n in loose if n.lower().endswith('.txt')]
            if directory_info:
                task_name = directory_info[0]
            elif len(txt_names) == 1:
                task_name = os.path.splitext(txt_names[0])[0]
            record = {"taskName": task_name, "taskId": "", "state": "READY",
                      "createdMs": int(min(os.path.getmtime(os.path.join(directory, n)) for n in loose) * 1000),
                      "objectId": self.robot_code + "-" + uuid.uuid4().hex,
                      "content": "\n\n".join(texts) or "启动补传图片（无TXT）",
                      "images": [n for n in loose if n.lower().endswith(image_extensions)], "hashes": {}}
            if directory_info:
                record.update(createdMs=directory_info[1], managedDirectory=True)
            for name in loose:
                with open(os.path.join(directory, name), "rb") as stream:
                    record["hashes"][name] = hashlib.sha256(stream.read()).hexdigest()
            path = os.path.join(directory, ".batch-" + uuid.uuid4().hex + ".json")
            base.atomic_write_json(path, record)
            base.rospy.loginfo('启动补传已发现: directory=%s 图片=%d 结果文件=%d；无文字也申请图片上传',
                               directory, len(record['images']), len(texts))
            self.outbox.enqueue_path(path)
        if not tracked and not loose:
            base.rospy.loginfo('目录没有待上传图片/TXT，不发送空启动请求: %s', directory)

    def shutdown(self):
        with self.lock:
            self.stopping = True
            if self.record is not None:
                self._save_record()
        self.outbox.stop()
        # 留下COLLECTING批次，下次启动等待任务状态后续采或封存。


def inspection_point_name(value):
    """从步骤名称提取点位；保留巡航/巡检前缀和原编号。"""
    match = re.search(r'巡[检航]点\s*([0-9]+|[一二三四五六七八九十百]+)', str(value or ''))
    return match.group(0) if match else ''


class SnapshotJieguoUploader(JieguoUploader):
    """当前入口：根目录单图持久化，沿用原凭证、PUT和回执协议。"""

    def __init__(self, args):
        args.robot_code = 'DT202600001'
        from plate_ocr import PlateOCR
        self.plate_ocr = PlateOCR.from_env()
        if self.plate_ocr is not None:
            base.rospy.loginfo('车牌OCR已启用：截图时识别，结果上传至files[].result中的车牌号')
        base.rospy.loginfo('截图上传版本=result-v4（files内携带result和pointName） source=%s', os.path.abspath(__file__))
        super().__init__(args)

    def _open_task_directory(self):
        self.task_dir = self.results_root
        self.record = None
        self.current_task_code = resolve_task_code(self.current_task_name)
        if getattr(self, 'point_task_identity', ''):
            self.last_passed_point = ''
            self.point_history = []
        self.point_task_identity = self.current_task_identity
        self.pending_move_point = ''
        self.active_collection = base.PointCollection(
            self.current_task_id, self.current_task_name,
            getattr(self, 'last_passed_point', ''), self.items, 1)
        self.last_snapshot = 0

    def _remember_point(self, point):
        if not point:
            return
        self.last_passed_point = point
        if not hasattr(self, 'point_history'):
            self.point_history = []
        if not self.point_history or self.point_history[-1] != point:
            self.point_history.append(point)
        if self.active_collection is not None:
            if self.active_collection.point_name != point:
                self.last_snapshot = 0
            self.active_collection.point_name = point

    def _close_current_task_locked(self, reason):
        self.active_collection = None
        self.record = None
        self.task_dir = ''

    def _point_callback(self, raw):
        try:
            message = base.json_load(raw)
            with self.lock:
                event = str(message.get('event', '')).lower()
                point = str(message.get('point_name') or '').strip()
                if event in ('arrived', 'leave'):
                    # 离点不能清空：随后20秒等待仍归属于刚经过的巡航点。
                    self._remember_point(point)
                    self.pending_move_point = ''
        except Exception as exc:
            base.rospy.logwarn_throttle(5, '截图点位解析失败: %s', exc)

    def _apply_robot_task(self, robot):
        applied = super()._apply_robot_task(robot)
        if applied and self.active_collection is not None:
            # 同一任务后续状态若省略编码，不覆盖已经收到的真实taskCode。
            self.current_task_code = resolve_task_code(
                self.current_task_name,
                robot.get('taskCode') or getattr(self, 'current_task_code', None))
            arg = robot.get('task_arg')
            arg = arg if isinstance(arg, dict) else {}
            task_type = str(robot.get('task_type') or arg.get('task_type') or '')
            explicit_point = str(robot.get('point_name') or arg.get('point_name') or '').strip()
            if task_type == 'ACTION_WAIT_TIME':
                point = (explicit_point or getattr(self, 'pending_move_point', '')
                         or inspection_point_name(robot.get('task_name'))
                         or getattr(self, 'last_passed_point', ''))
                self._remember_point(point)
                self.pending_move_point = ''
            elif task_type == 'TASK_MOVE_TO':
                # 到下一等待步骤时，上一个移动步骤对应的点已走过。
                self.pending_move_point = explicit_point or inspection_point_name(robot.get('task_name'))
        return applied

    def _collect_frame(self, frame):
        with self.lock:
            collection = self.active_collection
            if not self.task_active or collection is None:
                return
            collection.best_image = None
            collection.best_objects = []
            collection.best_score = 0
            # 单图识别项只来自当前帧，不能使用全任务累计的类别。
            collection.counts = {item['name']: 0 for item in self.items}
            # 截图上传置信度至少为0.6；保留原配置中更严格的阈值。
            self.min_confidence = max(0.6, self.min_confidence)
            base.YoloPointMqttUploader._collect_frame(self, frame)
            if (collection.best_image is None or not collection.best_objects
                    or time.monotonic() - self.last_snapshot < self.snapshot_interval):
                return
            if getattr(self, 'plate_ocr', None) is not None:
                self.plate_ocr.recognize_frame(collection.best_image, collection.best_objects)
            name = task_image_name(self.current_task_name, self.results_root)
            image = self._save_screenshot(collection, os.path.splitext(name)[0])
            if not image:
                return
            stamp = int(os.path.splitext(name)[0].rsplit('_', 1)[1])
            detected = []
            for item in self.items:
                if collection.counts[item['name']] <= 0:
                    continue
                # 目标检测没有文字时传空字符串；有识别正文则保留，不伪造车牌。
                contents = list(dict.fromkeys((str(obj.get('result_name') or item['display_name']),
                                              str(obj.get('content') or ''))
                    for obj in collection.best_objects if obj.get('canonical_name') == item['name']))
                for result_name, content in contents or [(item['display_name'], '')]:
                    detected.append(dict(name=result_name, content=content))
            metadata = dict(taskName=self.current_task_name, result=detected,
                            timestamp=stamp, pointName=collection.point_name,
                            robotCode=self.robot_code)
            with open(image, 'rb') as stream:
                digest = hashlib.sha256(stream.read()).hexdigest()
            record = dict(taskId=self.current_task_id, taskName=self.current_task_name,
                          taskCode=self.current_task_code,
                          objectId=self.robot_code + '-' + uuid.uuid4().hex,
                          createdMs=stamp, state='READY', singleImage=True,
                          images=[name], hashes={name: digest}, items=self.items,
                          counts=dict(collection.counts), imageMetadata=metadata)
            path = os.path.join(self.results_root, '.batch-' + uuid.uuid4().hex + '.json')
            base.atomic_write_json(path, record)
            self.outbox.enqueue_path(path)
            self.last_snapshot = time.monotonic()
            base.rospy.loginfo('单图已保存并入队: task=%s point=%s image=%s',
                               self.current_task_name, collection.point_name, name)
