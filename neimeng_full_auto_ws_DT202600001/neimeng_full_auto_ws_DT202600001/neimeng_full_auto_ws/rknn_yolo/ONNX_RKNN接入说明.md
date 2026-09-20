# ONNX 转 RKNN 与本项目接入

转换脚本：本目录 `onnx_to_rknn.py`，可单独复制到转换电脑使用，不依赖 ROS。
以下命令在 Linux 执行，项目内命令以 `neimeng_full_auto_ws` 为当前目录。

## 1. 转换环境

使用受 RKNN-Toolkit2 支持的 Linux 和 Python 版本，在独立虚拟环境安装转换依赖。按官方仓库安装与你的系统、Python、目标板 SDK 相匹配的版本；不要直接套用当前 Windows 的 Python 3.13 环境。

```bash
python3 -m venv .venv-rknn-convert
source .venv-rknn-convert/bin/activate
python3 -m pip install rknn-toolkit2 onnx
python3 -c 'from rknn.api import RKNN; import onnx; print("conversion dependencies OK")'
```

若提示没有匹配的发行包，按官方支持矩阵选择 Python，并安装官方提供的对应 wheel。转换用 `rknn-toolkit2`（`rknn.api.RKNN`）；板端现有程序用 `rknn-toolkit-lite2`（`rknnlite.api.RKNNLite`），两者用途不同。转换 SDK、板端 runtime 和 NPU 驱动需要使用兼容组合。

官方依据：[Toolkit2 安装与支持平台](https://github.com/airockchip/rknn-toolkit2)、[YOLOv8 转换示例](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolov8/python/convert.py)。本脚本采用 config → load_onnx → build → export_rknn 流程。

## 2. 转换你的模型

先生成非量化版本，方便对比原模型的检测结果：

```bash
python3 rknn_yolo/onnx_to_rknn.py /absolute/path/best.onnx \
  --output rknn_yolo/models/my_truck.rknn --target rk3588
```

脚本默认要求 ONNX 为单输入、静态 `[1,3,640,640]`，并打印输出名称及形状。动态输入请从训练/导出端重新导出固定尺寸。`--size` 用于校验其他正方形输入尺寸，不会自动改变模型；使用非 640 尺寸时还必须同步修改实际运行的 `rtsp_truck.py` 中 `MODEL_SIZE`。

本项目预处理为 letterbox → BGR 转 RGB → uint8 NHWC，代码不执行 `/255`。因此默认在转换时设置 `mean=[0,0,0]`、`std=[255,255,255]`。这要求原 ONNX 接收 RGB 的 0～1 输入；如果 ONNX 已经内置 `/255`，应核实后使用 `--std 1 1 1`，避免重复归一化。

需要 INT8 时，准备现场代表性图片，覆盖车辆、背景、光照和距离变化，建议先使用数百张。尽量按项目相同的 letterbox 方法生成 640×640 校准图片（填充值见 `letterbox()`），不要先除以 255。清单每行一个图片路径，相对路径以清单所在目录为基准，路径不要含空格：

```text
calibration/frame001.jpg
calibration/frame002.jpg
```

```bash
python3 rknn_yolo/onnx_to_rknn.py /absolute/path/best.onnx \
  --output rknn_yolo/models/my_truck_int8.rknn \
  --target rk3588 --quantize --dataset /absolute/path/dataset.txt
```

默认拒绝覆盖已有模型；确认要覆盖时加 `--force`。转换成功不代表后处理兼容或精度达标。

## 3. 配置模型路径：先确认你使用哪个入口

项目存在两套检测代码，改其中一套不会自动同步另一套。

### A. 原项目 rknn_yolo

将转换结果放到板端 `rknn_yolo/models/my_truck.rknn`。修改 `rknn_yolo/yolo.env` 的已有条目：

```bash
YOLO_BACKEND=rknn
RKNN_TRUCK_MODEL="${SCRIPT_DIR}/models/my_truck.rknn"
YOLO_CONF_THRESHOLD=0.35
```

`${SCRIPT_DIR}` 由 `run_yolo.sh` 设置，指向板端实际检测器目录。激活板端现有推理环境后启动：

```bash
conda activate rknn_yolo
bash rknn_yolo/run_yolo.sh rknn
```

若模型是标准 COCO 80 类，则设置 `YOLOV8_RKNN_MODEL` 指向新模型，用 `bash rknn_yolo/run_yolo.sh yolov8`。`run_rknn_truck.py` 固定选择 truck10 类别表；`run_yolov8.py` 固定选择 coco80。不要仅修改 `RKNN_MODEL`，这两个入口会重新赋值。

### B. 独立检测上传模块 independent_systems/yolo_upload_system

将模型放入 `independent_systems/yolo_upload_system/detector/models/my_truck.rknn`，在 `independent_systems/yolo_upload_system/config/yolo.env` 添加或修改：

```bash
RKNN_TRUCK_MODEL="${SCRIPT_DIR}/models/my_truck.rknn"
```

然后启动：

```bash
bash independent_systems/yolo_upload_system/run.sh rknn
```

这里的 `${SCRIPT_DIR}` 指向 `detector`。外层 `run.sh` 先设置默认模型路径，再由 `config/yolo.env` 覆盖，所以应改这个配置文件。后端由命令行参数选择。

如使用 COCO 模式，当前外层脚本会预先检查 `detector/models/yolov8n.rknn` 是否存在，因此最直接的配置是将对应 COCO 模型放到该位置，再执行 `run.sh yolov8`；只改 `YOLOV8_RKNN_MODEL` 不能绕过这一预检查。

## 4. 类别和输出必须与代码一致

原 10 类的固定顺序为：

```text
0 chocks, 1 tyre, 2 extinguisher, 3 plate, 4 tag,
5 light, 6 support, 7 screw, 8 tank, 9 truck
```

如果新模型训练 `data.yaml` 的 names 与上述完全相同，可直接用 `rknn` 入口。若类别数量或顺序不同，修改实际入口旁边的 `rtsp_truck.py`：`CLASSES` 按训练 class ID 顺序填写，`CLASS_NAMES_CN` 同步填写。原项目和独立模块各有一份该文件，两套都运行时要同步修改。`custom` 后端目前也是 truck10 入口，不能自动读取自定义类别。

检测上传还会按目标类别筛选：独立模块修改 `config/full_auto_yolo.yaml`；原 ROS 桥对应 `src/neimeng_bringup/config/full_auto_yolo.yaml`。现有配置 `target_class: "truck"`、`target_class_id: -1` 按名称识别卡车；若训练标签改名，应同步目标名称。若实际 ROS 启动入口使用其他 YAML，应修改它加载的配置。

现有 `post_process()` 仅实现以下两条路径：

- 单输出 `[1,4+类别数,N]` 或 `[1,N,4+类别数]`：前 4 个值为 xywh，后面是类别分数。10 类通常为 `[1,14,8400]`，COCO 通常为 `[1,84,8400]`（640 输入）。
- 特定 YOLOv8 九输出：三个尺度，每组按 DFL bbox、class、辅助 score 排列；代码读取每组前两项，要求 NCHW 且 class 通道数匹配。

仅输出数量相同不能证明兼容。含 NMS 的最终检测输出、YOLOv5 带 objectness 的输出、分割模型、六输出等需要另外适配解码。转换脚本不修改输出头，也不会自动生成正确的类别表。若从 YOLOv8 重新导出，应使用固定 batch=1、imgsz=640，并关闭动态尺寸和内置 NMS，然后核对输出语义。

## 5. 上板验证

先停止已有检测进程，避免 ZMQ 5566 端口冲突，再启动新模型。查看“RKNN模型加载成功”“RK3588 NPU初始化成功”以及首帧输出日志；不得出现“模型输出结构异常”“类别配置不匹配”等提示。用包含已知目标的画面对比 ONNX 与 RKNN 的框位置、标签、置信度；INT8 版本还应与非量化版本对比。最后确认 ROS 桥能收到目标 truck 的结果。

本次未发现工作区中可用的 ONNX 文件，因此未执行真实模型转换或 RK3588 板端验证；请用上述命令转换你的实际模型。
