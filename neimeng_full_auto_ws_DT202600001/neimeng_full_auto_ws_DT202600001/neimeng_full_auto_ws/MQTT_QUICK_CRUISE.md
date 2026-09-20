# 中台 MQTT 一键自动巡检

入口为 `run_all_with_point_upload.sh`。启动底盘、云台、YOLO、巡检编排器和结果上传后，接收程序等待中台发送 `quick_cruise`，不会仅因接收程序启动而下发行驶指令。

## 机器人端配置

在 Ubuntu 20.04 / ROS Noetic 机器人上进入本工作空间：

```bash
cp quick_cruise.env.example quick_cruise.env
nano quick_cruise.env
bash run_all_with_point_upload.sh rknn
```

`ROBOT_CODE` 必须与中台设备编码一致。`ROBOT_MQTT_HOST`、`ROBOT_MQTT_PORT` 统一用于巡检控制和结果上传；认证通过 `ROBOT_MQTT_USERNAME`、`ROBOT_MQTT_PASSWORD` 配置，未填写时沿用项目现有默认值。配置文件是本地 Bash 文件，仅填写可信配置。

默认从底盘发布的当前地图任务目录中选择唯一一个“任务0”。若有多个，设置 `QUICK_CRUISE_TASK='底盘上的完整任务0名称'`。不会根据中台任意字符串执行系统命令，也不会跳过现有自动巡检编排器。

可用 `MQTT_ENV_FILE=/绝对路径/配置.env` 指定配置文件。配置文件中的赋值优先于同名外部环境变量；不创建配置文件时可直接通过环境变量配置。ROS 工作空间环境默认使用 `devel_ubuntu20/setup.bash`，与底层启动脚本一致，可通过 `NEIMENG_DEVEL_DIR` 覆盖。

## 中台下发

订阅回执主题后，向 `thing/robot/DT202600001/services` 发布 UTF-8 JSON，建议 QoS 1，**retain 必须为 false**：

```json
{
  "tid": "每次新操作生成一个唯一UUID",
  "method": "quick_cruise",
  "timestamp": 1789696800000,
  "data": {"robotCode": "DT202600001"}
}
```

示例时间戳仅展示格式，发送时必须替换为当前 Unix 毫秒时间，且中台与机器人时间差不超过 120 秒。网络重试使用相同 `tid`；修正配置后重新发起操作使用新的 `tid`。

中台 Python 发布示例（运行后会真实下发启动请求）：

```python
import json
import os
import time
import uuid
from paho.mqtt.publish import single

robot = os.environ.get('ROBOT_CODE', 'DT202600001')
message = {
    'tid': str(uuid.uuid4()),
    'method': 'quick_cruise',
    'timestamp': int(time.time() * 1000),
    'data': {'robotCode': robot},
}
single(
    'thing/robot/{}/services'.format(robot),
    payload=json.dumps(message), qos=1, retain=False,
    hostname=os.environ['ROBOT_MQTT_HOST'],
    port=int(os.environ.get('ROBOT_MQTT_PORT', '1883')),
    auth={'username': os.environ['ROBOT_MQTT_USERNAME'],
          'password': os.environ['ROBOT_MQTT_PASSWORD']},
)
print(message['tid'])
```

## 回执与执行链路

回执发布至 `thing/robot/DT202600001/services_reply`，保留请求的 `tid` 和 `method`：

```json
{
  "tid": "与请求相同的UUID",
  "method": "quick_cruise",
  "timestamp": 1789696801000,
  "data": {
    "robotCode": "DT202600001",
    "result": 0,
    "msg": "车体启动指令处理成功，不代表巡检已完成"
  }
}
```

`result=0` 表示收到对应 `command_id` 的 `robot_bridge` 成功回执；`result=1` 表示拒绝、失败或执行状态未知，原因在 `msg`。30 秒内没有车体回执时报告状态未知，不自动重发。它是启动回执，不是全程巡检完成回执。

链路：MQTT `quick_cruise` → `/inspection/control` 的 `inspection.start` → 现有地图限定自动巡检编排器 → 底盘任务0 → 原有编号任务、云台和YOLO流程 → 原有 jieguo 上传程序。

接收程序检查巡检状态和任务目录的新鲜度、忙碌状态、控制话题订阅者及任务0唯一性，拒绝 retained 旧指令。SQLite 在下发前记录 `tid`，用于进程重启后去重；不要在重试或运行期间删除 `independent_systems/yolo_upload_system/quick_cruise_commands.sqlite3`。如果进程在下发和回执之间中断，同一 `tid` 只返回“执行状态待核查”，不会再次驱动车体。

本次接入只处理协议中的 `quick_cruise`。`task_upload`、`task_stop`、返航等其他方法不由该接收程序处理。退出启动脚本会停止软件进程，不等于向底盘发送停止指令；已经开始的底盘任务需使用现有巡检面板停止功能。

## 验证与排查

无需连接机器人即可运行接收逻辑测试：

```bash
python3 -m unittest discover -s independent_systems/yolo_upload_system/tests -v
```

正常启动日志应显示订阅 `thing/robot/<编码>/services` 成功。无法启动时检查回执的 `msg`，并在机器人端查看：

```bash
rostopic echo /inspection/robot_catalog
rostopic echo /inspection/status
rostopic echo /inspection/task_status
```

离线测试覆盖指令转换、真实回执匹配、持久化去重、错误设备过滤、retained 过滤、时间戳校验、任务歧义、状态超时、忙碌、启动失败及回执超时。离线通过不代表已完成现场 ROS、MQTT Broker 和底盘联调。
