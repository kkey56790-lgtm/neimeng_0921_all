# MQTT 文档格式版使用说明

运行 `robot_mqtt_all_in_one_task_queue.py`，停止旧综合服务和旧队列进程后再启动。原始 robot_mqtt_all_in_one.py 与单任务修复版未修改。

本版按《（车体）MQTT通信协议(2)》恢复任务列表、任务下发与回执格式。内部顺序队列保留；不再接受批量数组和 tasks 数组，不再发送 task_finished、stage、eventId、taskId、taskIndex、task_code、taskQueue 等扩展字段。原自定义 get_current_map_tasks 查询指令停用。

新增 task_switch 是按现场需求添加的协议扩展。三种操作：task_upload 追加到队尾；task_stop 停止当前并清空等待列表；task_switch 停止当前并把指定任务插入队首，保留原等待列表。已被切换掉的当前任务不自动重新入队。

## 立即切换任务

Topic 同为 `thing/robot/DT202600001/services`，例如：

```json
{"tid":"switch-7-001","method":"task_switch","timestamp":1789782000000,"data":{"robotCode":"DT202600001","mainTaskName":"任务7","taskCode":"TASK-2D75B27BE1D1EBE2"}}
```

先检查目标任务存在、名称编码一致，再停止旧任务并等待停止确认。无效目标不会打断旧任务；停止失败保留原队列并阻塞，不启动目标任务。确认目标启动后，回执 method=task_switch、data={"result":0}；完成后 method=task_switch、data={"mainTaskName":"任务7","result":0}，tid 都保留切换指令的 tid。切换失败返回同 method、result=-1，原因见车机日志。切换后任务若执行失败，仍按现有规则取消剩余等待任务。

例：当前 A，等待 B、C，下发切换 D 后执行顺序为 D、B、C；A 会收到原 tid 对应的取消结果 result=-1。

## 任务下发

Topic：`thing/robot/DT202600001/services`，retain=false。每个任务单独发送，使用不同 tid，按到达顺序排队。timestamp 替换成实际发送时的毫秒时间戳。

```json
{
  "tid": "task-7-001",
  "method": "task_upload",
  "timestamp": 1789782000000,
  "data": {
    "robotCode": "DT202600001",
    "mainTaskName": "任务7",
    "taskCode": "TASK-2D75B27BE1D1EBE2"
  }
}
```

robotCode、mainTaskName、taskCode 都必须为非空字符串，名称和编码须对应同一底盘主任务。不接受 taskCode 数组或数组文本。多个任务依次发送多条上述消息，不打断当前任务。

允许中台附带额外字段，但只使用上述三个字段控制任务。

## 开始与完成回执

Topic：`thing/robot/DT202600001/services_reply`，tid 为原指令 tid。

确认开始时：

```json
{"tid":"task-7-001","method":"task_upload","timestamp":1789782000100,"data":{"result":0}}
```

确认完成时：

```json
{"tid":"task-7-001","method":"task_upload","timestamp":1789782100000,"data":{"mainTaskName":"任务7","result":0}}
```

这是文档规定的两种格式。完成回执不再使用 task_finished。按文档回执中不额外添加 robotCode，机器人编号已包含在 Topic 中。失败用 result=-1，详细原因只打印车机日志。入队和临时状态未知没有额外回执；不能将 result=0 的开始回执误当成完成回执。每个任务使用独立 tid 关联。

## 任务列表上报

Topic：`thing/robot/DT202600001/osd`，method=`task_list`。

data 仅包含 robotCode、robotName、taskList、currentTask、taskListError。

taskList 每项仅包含 taskCode、mainTaskName、index、mapNames、subTaskCount。

currentTask 保留文档字段：error_info、local_state、main_task_name、map、obstacle、queue_size、remai_distance、remai_loop、remai_time、task_name、task_state、task_time、task_type、taskCode；没有取得底盘字段时不伪造，状态读取失败时 currentTask 为空且 taskListError 提示异常。没有当前主任务时 taskCode 为空。

当前巡航点名在 currentTask.task_name，主任务名在 currentTask.main_task_name。底盘任务列表未携带编码时仍沿用原有名称生成编码的规则。

任务/地图列表上报周期为 60 秒；实际频率受 HTTP 请求耗时影响。基础 OSD 状态上报周期仍为约 1 秒。平台订阅 osd 接收周期列表，不再发送自定义地图查询指令。

## 停止、异常和重启

停止消息：

```json
{"tid":"stop-001","method":"task_stop","timestamp":1789782000000,"data":{"robotCode":"DT202600001"}}
```

task_stop 清空等待列表并停止当前任务，确认后返回 method=task_stop、data={"result":0}。停止失败返回 result=-1 并阻塞后续调度。任务失败时取消剩余等待项；排队任务取消时通过其原 tid 返回 mainTaskName 和 result=-1。

机器人端兼容固定停止 tid：task_stop 不参与去重，即使每次使用 cmd-uuid-004，或该 tid 已用于其他指令，也会重新执行停止并返回本次结果；不会重放上一次成功回执。重复停止会作用于收到时正在运行的任务并清空当时的等待列表。task_upload 和 task_switch 仍按 tid 去重。robotCode 校验及拒绝 retained 控制消息的规则不变。

同一主任务的 task_time 变化不会阻塞任务完成判定；状态读取失败或读到其他主任务不会被当作完成。

队列、去重记录及断线期间待发回执只存在内存。程序重启后丢失，不恢复运行，也不主动停止底盘。正常重启前先下发 task_stop，等待成功回执后再退出。重新下发使用新 tid。保留最多约 1000 个已处理指令的去重记录，活动任务的记录不会被淘汰。

运行：

```bash
python3 -u robot_mqtt_all_in_one_task_queue.py
```

离线验证，不连接实车：

```bash
python3 test_robot_mqtt_task_queue.py
```
