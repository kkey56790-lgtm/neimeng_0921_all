# 单张识别截图上传

原启动入口 `scripts/mqtt_yolo_point_uploader.py` 现在使用 `SnapshotJieguoUploader`。
启动命令和 MQTT 配置沿用现有设置，机器人编码固定为 `DT202600001`。

每次检出目标后，按原截图间隔生成一张 JPG 及一份 `.batch-*.json` 附带信息，直接放在 `--results-root`（默认 jieguo）根目录。每张图片单独立即进入上传队列，不再按任务新建目录，不再等待任务结束生成 TXT。每张图有独立 objectId；识别项目来自该图对应检测帧，不沿用任务历史累计结果。

MQTT 主题、`file_upload_request`、`files` 列表、签名 HTTP PUT、`file_upload_reply` 和确认后清理流程不变。上传请求的 `data` 携带以下信息，识别结果与点位放在每个 `files` 元素内，原来的独立 `content` 字段已删除：

```json
{
  "taskName": "一区巡检",
  "timestamp": 1790000000000,
  "robotCode": "DT202600001",
  "files": [{
    "fileName": "一区巡检_1790000000000.jpg",
    "fileSize": 41568,
    "mimeType": "image/jpeg",
    "mediaType": 1,
    "md5": "图片实际MD5",
    "result": [{"name": "轮胎掩护", "content": ""}, {"name": "车牌号", "content": "B1001"}],
    "pointName": "巡航点2"
  }]
}
```

示例车牌仅用于说明字段格式，不会写死到程序。请求中不再发送 `data.result`、`data.pointName`；各图片分别使用自己的 `files[i].result` 和 `files[i].pointName`。


`data.timestamp` 为截图的 Unix 毫秒时间戳，重试时不变；外层 `timestamp` 仍为本次协议消息的发送时间。`result[].content` 为字符串；普通目标检测无正文时为空，有实际识别文字时保留该文字。旧待传记录里的 `detectionItems/name/count` 发送时也转换为新格式。车体发往中台的 `file_upload_reply` 回执同样携带 `result: [{"name": "车灯破损", "content": ""}]`，不再使用数字 `result: 0`，也不添加 `resultCode`。中台发给车体的凭证响应 `file_upload_response` 仍按原协议解析。

点位读取 `/inspection/point_event` 的到点、离点事件，并在内存记录经过的点位；保留实际巡航点原名称，例如 `巡航点04`。`/inspection/task_status` 的 `TASK_MOVE_TO` 步骤记录点位，进入后续 `ACTION_WAIT_TIME`（例如等待20s）时沿用前一个移动步骤的点位。也支持等待步骤直接携带 `point_name`、`task_arg.point_name` 或含点位名的 `task_name`。离点和移动不再清空最近经过的点位，切换主任务时清除上一任务记录。

截图不再等待点位信息：如果进程启动时尚未收到任何有效点位，仍保存并上传，`pointName` 暂为空；后续收到点位后，新图片自动携带点位名。已入队图片的信息保持截图时的快照，不用未来点位回填。

重启逐张恢复未完成图片；旧版已存在任务目录继续按旧记录补传，不迁移或猜测历史图片点位。原累计三次上传失败清理策略保留，单图模式只清理该图片及其批次 JSON，绝不删除结果根目录或其他图片。

离线回归：`python3 -m unittest discover -s independent_systems/yolo_upload_system/tests -v`。
需在机器人上重启原上传进程后生效；离线测试不代表已完成摄像头、ROS 和中台联调。

排查中台看不到 `result`：更新机器人上的 `scripts/jieguo_uploader.py`（入口仍使用此前修改后的 `mqtt_yolo_point_uploader.py`），重启后应看到 `截图上传版本=result-v4…source=...`。每次实际请求都会打印 `MQTT上传请求原文[result-v2]`，内容与传入 MQTT publish 的字符串完全一致，按同一个 tid 与中台消息对照。发送前统一保证请求的 `data.files[i].result` 为数组，兼容旧 `detectionItems/count` 和旧文字 `content`；没有识别信息时发送空数组，不捏造结果。如果该原文的 files 内有 result 而中台处理后的日志没有，需要排查中台字段映射。

车体回执实际发送原文日志为 `MQTT车体回执原文[result-v3]`，可按 tid 核对中台收到的 result 数组。
