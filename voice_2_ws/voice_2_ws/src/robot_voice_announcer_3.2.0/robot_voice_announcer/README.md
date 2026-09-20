# 机器人 HTTP 语音管理与状态播报包

当前版本 3.2.0。MQTT 最新部署和协议说明见 [使用说明_3.2.md](使用说明_3.2.md)。
MQTT 默认语音已改为中文文件名；默认无限等待；同名始终覆盖；更新、查询、播放均返回 data.result。
下文 HTTP 安装工具使用的英文 WAV 文件名与 MQTT 文字生成方案分开使用。

这是独立的 ROS 1 Noetic 包，不修改原巡检、导航和底盘代码。语音管理和播放均使用
《室内外智能导航 HTTP 协议 V2.4》的 7999 端口接口。

## 主要程序

- `install_voice_files.py`：获取远程列表，下载备份全部旧语音，逐个删除，再上传六个 WAV。
- `check_robot_voice_http.py`：只读检查远程六个文件，可下载到内存验证 WAV 格式。
- `robot_voice_monitor.py`：读取底盘和任务状态，通过远程播放接口播报。

## HTTP 接口

- `GET /system_manager/get_wav_list`
- `POST /system_manager/download_wav?file_name=...`
- `POST /system_manager/delete_wav_data`，JSON 为 `{"file_name":"..."}`
- `POST /system_manager/upload_wav_data?file_name=...`，Body 为 WAV 原始字节
- `POST /system_manager/play_wav_data`，JSON 为 `{"file_name":"..."}`
- `GET /sensor_data/base_data`
- `GET /task_manager/get_task_status`

协议示例显示服务端目录为
`/home/robotcar/catkin_http/src/Control/ros_speek_service/wav/`，本包无需直接操作该目录。

## 更新语音

```bash
rosrun robot_voice_announcer install_voice_files.py --robot-ip 192.168.2.100 --dry-run
rosrun robot_voice_announcer install_voice_files.py --robot-ip 192.168.2.100
```

第二条命令先将全部旧语音下载到当前目录下带时间戳的备份文件夹，再删除旧语音、
上传六个新文件并读取服务器列表验证。

当前机器人固件禁止通过 HTTP 删除 `turn_left.wav`、`turn_right.wav` 和
`stop_car.wav`。程序会完整备份并保留这三个系统安全提示音，删除其他可删除语音，
新语音使用不同文件名，不会与它们冲突。

## 检查远程语音

```bash
rosrun robot_voice_announcer check_robot_voice_http.py \
  --robot-ip 192.168.2.100 --download-check
```

## 启动状态播报

```bash
roslaunch robot_voice_announcer robot_voice_monitor.launch robot_ip:=192.168.2.100
```

节点读取 `/sensor_data/base_data` 的 `nav.obstacle`、`robot.vx`、`robot.vz`，以及
`/task_manager/get_task_status` 的 `task_state`、`obstacle`。判断优先级为避障、倒车、
转向、巡检或前进、停车。HTTP 失联时发布 `unknown`，不发送播放请求。

## MQTT 语音桥

MQTT 主题遵循车体协议：

- 订阅 `thing/robot/{robotCode}/services`
- 订阅 `thing/robot/{robotCode}/osd`
- 发布 `thing/robot/{robotCode}/services_reply`

支持 `voice_package_update`、`voice_package_list`、`voice_switch`。语音包更新消息中的
文字通过机器人 HTTP `create_wav_data` 接口生成 WAV。OSD 状态用于自动播报。

中台上传消息示例：

```json
{
  "tid": "wav-upload-001",
  "method": "voice_package_update",
  "timestamp": 1789354628341,
  "data": {
    "robotCode": "DT202600001",
    "overwrite": true,
    "voices": [
      {
        "name": "正在巡检中请勿靠近.wav",
        "text": "正在巡检中请勿靠近",
        "type": "Chinese"
      }
    ]
  }
}
```

`turn_left.wav`、`turn_right.wav`、`stop_car.wav` 为固件保护文件，不能通过更新命令覆盖。

MQTT 账号密码已经按 1 号车参数写入包内配置文件，无需再复制凭据文件。包应放在：

```bash
~/voice_1_ws/src/robot_voice_announcer
```

启动机器人端桥接：

```bash
roslaunch robot_voice_announcer mqtt_voice_bridge.launch
```

在另一个终端发送六条默认语音包更新：

```bash
rosrun robot_voice_announcer publish_voice_package.py \
  --robot-code DT202600001 --update
```

获取列表或测试播放：

```bash
rosrun robot_voice_announcer publish_voice_package.py --robot-code DT202600001 --list
rosrun robot_voice_announcer publish_voice_package.py --robot-code DT202600001 \
  --play '正在巡检中请勿靠近.wav'
```
