模型文件：

- `yolov8n.rknn`：标准YOLOv8 COCO 80类，使用 `bash rknn_yolo/run_yolo.sh yolov8`。
- `best_truck.rknn`：原10类truck模型，使用 `bash rknn_yolo/run_yolo.sh rknn`。

两个入口互斥启动，只发布检测JSON，不向ROS/UI发送视频。