# 云台随动独立系统

默认自带车体桥接，根据真实位姿产生到点事件：

```bash
bash independent_systems/ptz_follow_system/run.sh
```

与任务切换模块一起运行时，关闭本模块的重复车体桥接：

```bash
bash independent_systems/ptz_follow_system/run.sh enable_robot_bridge:=false
```

规则位于 `config/ptz_points.yaml`：初始点→预置位1，起始点01-N→预置位2，巡航点→预置位3并巡航，预置位4保留接口。新固定点会取消旧巡航；巡航在预置位动作完成后执行。

```mermaid
flowchart TD
  A["车体位姿和真实巡航点"] --> B["生成arrived到点事件"]
  B --> C["按精确点名或关键词匹配"]
  C --> D["切换云台auto模式"]
  D --> E{"点位类型"}
  E -->|初始点| F["最快到预置位1"]
  E -->|起始点01-N| G["最快到预置位2"]
  E -->|巡航点| H["最快到预置位3"]
  E -->|预置位4接口| I["到预置位4"]
  H --> J["巡航进入保护队列"]
  J --> K["预置位完成后快速巡航"]
  B -->|新点位到达| L["取消旧巡航并抢占新预置位"]
```
