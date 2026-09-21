# 车牌OCR接入（最少修改现有程序）

只部署 `scripts/plate_ocr.py` 和更新后的 `scripts/jieguo_uploader.py`，放在原 `mqtt_yolo_point_uploader.py` 同一目录。检测器、模型、ROS主题、MQTT凭证和上传回执均保持原流程。提供的附件保留不动，适配后的副本为 `scripts/plate_ocr.py`。

视频持续检测 → 原上传器挑选待截图帧 → 取已有车牌框 → EasyOCR识别 → 保存同一张图片 → 原流程上传。OCR仅在满足截图间隔且YOLO置信度≥0.6时运行，不对所有视频帧重复计算。无需第二套YOLO、Ultralytics或best.pt；也不全局修改torch.load。

当前十类模型车牌名称为 `plate`、ID为3；按名称匹配，不使用附件的ID 2（当前模型的灭火器）。标准COCO80没有车牌类，必须使用能够检测车牌的现有自定义模型，类别顺序必须与训练一致。

## 1. 在上传器实际使用的Python环境安装

下面的python3必须与启动 `mqtt_yolo_point_uploader.py` 使用的是同一个解释器；不要仅在检测器的conda环境安装，而上传器使用另一个Python。

```bash
python3 -m pip install easyocr -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -c "import cv2, easyocr, rospy; print('依赖检查通过')"
```

EasyOCR依赖PyTorch/torchvision，机器人ARM系统需要与其Python、架构兼容的包；如安装报错，保留完整报错以及 `python3 --version`、`uname -m` 信息排查，不使用附件的全局torch.load参数删除补丁。

## 2. 预先初始化模型

在 `yolo_upload_system` 目录执行（首次需要网络下载英文识别权重）：

```bash
PLATE_OCR_ENABLED=1 PYTHONPATH="$PWD/scripts:$PYTHONPATH" python3 -c "from plate_ocr import PlateOCR; PlateOCR.from_env(); print('OCR模型就绪')"
```

默认EasyOCR模型目录为当前运行用户的 `~/.EasyOCR/model`。离线部署可先在联网环境下载模型文件并复制到机器人，设置 `PLATE_OCR_MODEL_DIR` 为其绝对目录，另设 `PLATE_OCR_DOWNLOAD=0`。服务用户必须能读取该目录。

## 3. 启动

先保持原检测程序运行。在启动上传器的同一个终端执行：

```bash
export PLATE_OCR_ENABLED=1
export PLATE_OCR_MIN_CONFIDENCE=0.5
# 可选：export PLATE_OCR_MODEL_DIR=/home/mile/.EasyOCR/model
python3 scripts/mqtt_yolo_point_uploader.py
```

最后一行继续带上你原来的 MQTT、RTSP、results-root 等参数。若使用systemd或其他启动脚本，将环境变量加到实际启动上传器的服务/脚本中，仅放到检测器yolo.env不会传给独立的上传进程。`PLATE_OCR_ENABLED=0`关闭OCR，恢复原截图行为。启用时依赖或模型缺失会在启动时报错，应先通过步骤2再重启现场服务。

## 4. 中台数据

```json
"files": [{
  "fileName": "任务8_1789800606590.jpg",
  "result": [{"name": "车牌号", "content": "B1001"}],
  "pointName": "巡航点2"
}]
```

其余文件属性照旧。无可用OCR文字时content为空，绝不沿用上一辆车的车牌；保留同图其他检测项。当前与附件一致，只识别英文字母和数字，不包含中文省份简称。OCR置信度门槛默认0.5，与YOLO截图门槛0.6分别生效。

最少改动方案保留现有“ZMQ检测框 + 独立RTSP截图帧”机制：OCR对最终截图图像裁剪识别，不使用另一张图片的OCR缓存；但原有框与截图流并非严格同帧，在快速移动时可能存在框偏移。需现场验证清晰度、框位置及CPU耗时。若要求检测与截图严格同帧，需要另行改造检测端传递图像，本次按要求未修改检测程序。

参考：https://www.jaided.ai/easyocr/documentation/
