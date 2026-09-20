# 任务切换独立系统

只负责车体HTTP桥接、当前地图任务过滤、编号任务循环、回充、障碍和轻量UI。默认不依赖云台和YOLO。

```bash
bash independent_systems/task_switch_system/run.sh
```

与独立YOLO组合时：

```bash
bash independent_systems/task_switch_system/run.sh enable_yolo:=true
```

任务名必须包含“任务N”，例如“西任务1台位”。当前地图有任务1、2、3时按 1→2→3→1 循环；若没有任务1，末尾回到最小真实编号。

```mermaid
flowchart TD
  A["读取当前地图和车体任务列表"] --> B["过滤当前地图真实任务"]
  B --> C["解析任务名中的任务N"]
  C --> D["启动当前编号任务"]
  D --> E{"收到匹配任务的STATE_FINISH"}
  E -->|否| D
  E -->|是| F{"回充或平台队列待执行"}
  F -->|低电量| G["执行真实回充任务"]
  F -->|平台任务| H["执行队首平台任务"]
  F -->|均无| I["选择下一个真实编号"]
  G --> J["电量达到目标后恢复"]
  H --> E
  I --> D
  J --> D
```
