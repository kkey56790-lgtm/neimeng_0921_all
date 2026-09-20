#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

import cv2
import time
import json
import zmq
import threading
import numpy as np

from datetime import datetime
from urllib.parse import quote
from rknnlite.api import RKNNLite


# ============================================================
# RKNN配置
# ============================================================

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
# 有 best.rknn 时优先运行用户提供的标准 YOLOv8 模型；没有时才兼容旧模型。
_YOLOV8_MODEL = os.path.join(_SCRIPT_DIR, "models", "best.rknn")
_LEGACY_MODEL = os.path.join(_SCRIPT_DIR, "models", "best_truck.rknn")
RKNN_MODEL = os.environ.get("RKNN_MODEL", _YOLOV8_MODEL if os.path.isfile(_YOLOV8_MODEL) else _LEGACY_MODEL)
DEFAULT_CLASS_PROFILE = "coco80" if os.path.basename(RKNN_MODEL) == "best.rknn" else "truck10"

MODEL_SIZE = 640

CONF_THRESHOLD = float(os.environ.get("YOLO_CONF_THRESHOLD", "0.5"))

NMS_THRESHOLD = 0.45

MAX_DETECTIONS = 50


# ============================================================
# ZMQ结果发布
#
# 检测程序负责 bind
#
# 后面的触发程序负责 connect
# ============================================================

ZMQ_PUB_ADDRESS = os.environ.get("YOLO_ZMQ_ADDRESS", "tcp://127.0.0.1:5566")

# 无桌面部署时设置 YOLO_DISPLAY=0，避免 cv2.imshow 依赖 DISPLAY。
YOLO_DISPLAY = os.environ.get("YOLO_DISPLAY", "1").lower() in ("1", "true", "yes")


# ============================================================
# 10类
#
# 必须与训练 data.yaml 完全一致
# ============================================================

CLASSES = [
    "chocks",        # 0 挡掩
    "tyre",          # 1 轮胎
    "extinguisher",  # 2 灭火器
    "plate",         # 3 车牌
    "tag",           # 4 检修牌
    "light",         # 5 车灯
    "support",       # 6 支护
    "screw",         # 7 轮毂螺丝
    "tank",          # 8 油箱
    "truck"          # 9 卡车
]


# 设置 YOLO_CLASS_PROFILE=coco80 时，使用现场提供 YOLOv8 COCO 模型的类别表。
COCO_CLASSES = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
CLASS_NAMES_CN = [
    "挡掩",
    "轮胎",
    "灭火器",
    "车牌",
    "检修牌",
    "车灯",
    "支护",
    "轮毂螺丝",
    "油箱",
    "卡车"]

# 默认为原 10 类模型；使用你提供的 COCO YOLOv8 模型时在 yolo.env 中设置
# YOLO_CLASS_PROFILE=coco80。桥接按名称 truck 识别，因此不需要把 YAML 的类别编号改为 7。
if os.environ.get("YOLO_CLASS_PROFILE", DEFAULT_CLASS_PROFILE).strip().lower() in ("coco", "coco80", "yolov8"):
    CLASSES = COCO_CLASSES
    CLASS_NAMES_CN = COCO_CLASSES


# ============================================================
# RTSP 最新帧读取
# ============================================================

class LatestFrameReader:

    def __init__(self, url):

        self.url = url

        self.cap = None

        self.frame = None

        self.frame_id = 0

        self.running = False

        self.thread = None

        self.lock = threading.Lock()


    def open(self):

        if self.cap is not None:
            self.cap.release()

        print("正在打开 RTSP...")

        self.cap = cv2.VideoCapture(
            self.url,
            cv2.CAP_FFMPEG
        )

        if not self.cap.isOpened():

            print("RTSP连接失败")

            self.cap = None

            return False

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        print("RTSP连接成功")

        return True


    def start(self):

        self.running = True

        self.thread = threading.Thread(
            target=self.loop,
            daemon=True
        )

        self.thread.start()


    def loop(self):

        while self.running:

            if (
                self.cap is None
                or
                not self.cap.isOpened()
            ):

                if not self.open():

                    time.sleep(2)

                    continue

            ret, frame = self.cap.read()

            if not ret:

                print(
                    "RTSP读取失败，重新连接..."
                )

                if self.cap is not None:
                    self.cap.release()

                self.cap = None

                time.sleep(0.5)

                continue

            with self.lock:

                self.frame = frame

                self.frame_id += 1


    def read_latest(self, last_id):

        with self.lock:

            if self.frame is None:

                return last_id, None

            if self.frame_id == last_id:

                return last_id, None

            return (
                self.frame_id,
                self.frame.copy()
            )


    def stop(self):

        self.running = False

        if self.cap is not None:
            self.cap.release()

        if self.thread is not None:

            self.thread.join(
                timeout=1.0
            )


# ============================================================
# RTSP 地址
# ============================================================

def build_rtsp_url():

    username = quote(
        os.environ.get("RTSP_USERNAME", "admin"),
        safe=""
    )

    password = quote(
        os.environ.get("RTSP_PASSWORD", "okwy1688"),
        safe=""
    )

    ip = os.environ.get("RTSP_HOST", "192.168.2.64")

    port = int(os.environ.get("RTSP_PORT", "554"))

    channel = os.environ.get("RTSP_CHANNEL", "102")

    return (
        f"rtsp://{username}:{password}"
        f"@{ip}:{port}"
        f"/Streaming/Channels/{channel}"
    )


# ============================================================
# LetterBox
# ============================================================

def letterbox(image):

    h, w = image.shape[:2]

    scale = min(
        MODEL_SIZE / w,
        MODEL_SIZE / h
    )

    new_w = int(
        round(w * scale)
    )

    new_h = int(
        round(h * scale)
    )

    resized = cv2.resize(
        image,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR
    )

    canvas = np.zeros(
        (
            MODEL_SIZE,
            MODEL_SIZE,
            3
        ),
        dtype=np.uint8
    )

    dx = (
        MODEL_SIZE - new_w
    ) // 2

    dy = (
        MODEL_SIZE - new_h
    ) // 2

    canvas[
        dy:dy + new_h,
        dx:dx + new_w
    ] = resized

    return (
        canvas,
        scale,
        dx,
        dy
    )


# ============================================================
# RKNN输入预处理
#
# 转换模型：
#
# mean_values=[[0,0,0]]
# std_values=[[255,255,255]]
#
# 因此这里：
#
# RGB
# uint8
# NHWC
#
# 不 /255
# ============================================================

def preprocess(frame):

    image, scale, dx, dy = letterbox(
        frame
    )

    image = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2RGB
    )

    image = np.ascontiguousarray(
        image,
        dtype=np.uint8
    )

    image = np.expand_dims(
        image,
        axis=0
    )

    return (
        image,
        scale,
        dx,
        dy
    )


# ============================================================
# sigmoid
# ============================================================

def sigmoid(x):

    x = np.clip(
        x,
        -50.0,
        50.0
    )

    return (
        1.0
        /
        (
            1.0
            +
            np.exp(-x)
        )
    )


# ============================================================
# NMS
# ============================================================

def nms_boxes(
    boxes,
    scores
):

    if len(boxes) == 0:

        return np.array(
            [],
            dtype=np.int32
        )

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]

    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    widths = np.maximum(
        0.0,
        x2 - x1
    )

    heights = np.maximum(
        0.0,
        y2 - y1
    )

    areas = (
        widths
        *
        heights
    )

    order = scores.argsort()[::-1]

    keep = []

    while order.size > 0:

        i = order[0]

        keep.append(i)

        if order.size == 1:
            break

        xx1 = np.maximum(
            x1[i],
            x1[order[1:]]
        )

        yy1 = np.maximum(
            y1[i],
            y1[order[1:]]
        )

        xx2 = np.minimum(
            x2[i],
            x2[order[1:]]
        )

        yy2 = np.minimum(
            y2[i],
            y2[order[1:]]
        )

        inter_w = np.maximum(
            0.0,
            xx2 - xx1
        )

        inter_h = np.maximum(
            0.0,
            yy2 - yy1
        )

        inter = (
            inter_w
            *
            inter_h
        )

        iou = (
            inter
            /
            (
                areas[i]
                +
                areas[order[1:]]
                -
                inter
                +
                1e-6
            )
        )

        inds = np.where(
            iou <= NMS_THRESHOLD
        )[0]

        order = order[
            inds + 1
        ]

    return np.asarray(
        keep,
        dtype=np.int32
    )


# ============================================================
# YOLO单输出后处理
#
# 10类：
#
# (1,14,8400)
#
# 14 = 4 bbox + 10 classes
# ============================================================

# 标准 YOLOv8 RKNN 导出模型通常是 9 个输出：3 个尺度的 bbox/class/score。
# 该解码路径与现场提供的 COCO YOLOv8 代码一致；检测结果仍按现有 ZMQ 格式发送给巡检系统。
def _softmax(x, axis=2):
    x = x - np.max(x, axis=axis, keepdims=True)
    exp_x = np.exp(x)
    return exp_x / np.sum(exp_x, axis=axis, keepdims=True)


def _decode_dfl_boxes(position):
    if position.ndim != 4 or position.shape[1] % 4 != 0:
        return None
    _, channels, grid_h, grid_w = position.shape
    bins = channels // 4
    position = position.reshape(1, 4, bins, grid_h, grid_w)
    position = (_softmax(position, axis=2) * np.arange(bins, dtype=np.float32).reshape(1, 1, bins, 1, 1)).sum(axis=2)
    col, row = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
    grid = np.concatenate((col.reshape(1, 1, grid_h, grid_w), row.reshape(1, 1, grid_h, grid_w)), axis=1).astype(np.float32)
    stride = np.array([MODEL_SIZE / grid_w, MODEL_SIZE / grid_h], dtype=np.float32).reshape(1, 2, 1, 1)
    xy1 = (grid + 0.5 - position[:, :2]) * stride
    xy2 = (grid + 0.5 + position[:, 2:]) * stride
    return np.concatenate((xy1, xy2), axis=1)


def _post_process_yolov8_nine_outputs(outputs):
    boxes_all, scores_all = [], []
    for scale_index in range(3):
        boxes = _decode_dfl_boxes(np.asarray(outputs[scale_index * 3], dtype=np.float32))
        scores = np.asarray(outputs[scale_index * 3 + 1], dtype=np.float32)
        if boxes is None or scores.ndim != 4 or scores.shape[1] != len(CLASSES):
            print("YOLOv8 9输出与类别配置不匹配:", np.asarray(outputs[scale_index * 3]).shape, scores.shape, "类别数:", len(CLASSES))
            return []
        boxes_all.append(boxes.transpose(0, 2, 3, 1).reshape(-1, 4))
        scores_all.append(scores.transpose(0, 2, 3, 1).reshape(-1, len(CLASSES)))
    boxes = np.concatenate(boxes_all, axis=0)
    class_scores = np.concatenate(scores_all, axis=0)
    class_ids = np.argmax(class_scores, axis=1)
    confidences = np.max(class_scores, axis=1)
    mask = confidences >= CONF_THRESHOLD
    boxes, class_ids, confidences = boxes[mask], class_ids[mask], confidences[mask]
    detections = []
    for class_id in np.unique(class_ids):
        indexes = np.where(class_ids == class_id)[0]
        for kept_index in nms_boxes(boxes[indexes], confidences[indexes]):
            source_index = indexes[kept_index]
            detections.append((boxes[source_index], int(class_id), float(confidences[source_index])))
    return sorted(detections, key=lambda item: item[2], reverse=True)[:MAX_DETECTIONS]

def post_process(
    outputs,
    debug=False
):

    if outputs is None:
        return []

    if len(outputs) == 9:
        return _post_process_yolov8_nine_outputs(outputs)

    if len(outputs) != 1:

        print(
            "模型输出数量异常:",
            len(outputs)
        )

        return []

    output = np.asarray(
        outputs[0],
        dtype=np.float32
    )

    if output.ndim == 3:

        output = output[0]

    elif output.ndim != 2:

        print(
            "模型输出维度异常:",
            output.shape
        )

        return []

    expected_channels = (
        4
        +
        len(CLASSES)
    )

    # --------------------------------------------------------
    # (14,8400)
    #
    # ->
    #
    # (8400,14)
    # --------------------------------------------------------

    if (
        output.shape[0]
        ==
        expected_channels
    ):

        output = output.T

    elif (
        output.shape[1]
        ==
        expected_channels
    ):

        pass

    else:

        print()

        print(
            "模型输出结构异常:"
        )

        print(
            "实际:",
            output.shape
        )

        print(
            "类别数:",
            len(CLASSES)
        )

        print(
            "期望channel:",
            expected_channels
        )

        return []


    boxes_xywh = output[
        :,
        0:4
    ]

    class_scores = output[
        :,
        4:
    ]


    # ========================================================
    # 类别数量检查
    # ========================================================

    if (
        class_scores.shape[1]
        !=
        len(CLASSES)
    ):

        print(
            "模型类别数量错误"
        )

        return []


    # ========================================================
    # Debug
    # ========================================================

    if debug:

        print()

        print(
            "===== OUTPUT DEBUG ====="
        )

        print(
            "bbox min/max:",
            float(
                boxes_xywh.min()
            ),
            float(
                boxes_xywh.max()
            )
        )

        print(
            "class min/max/mean:",
            float(
                class_scores.min()
            ),
            float(
                class_scores.max()
            ),
            float(
                class_scores.mean()
            )
        )

        print()

        print(
            "各类别最高置信度:"
        )

        for i, name in enumerate(
            CLASSES
        ):

            max_score = float(
                class_scores[:, i].max()
            )

            print(
                f"{i:2d}: "
                f"{name:15s} "
                f"{max_score:.6f}"
            )

        print(
            "========================"
        )


    # ========================================================
    # 判断是否需要 sigmoid
    # ========================================================

    score_min = float(
        class_scores.min()
    )

    score_max = float(
        class_scores.max()
    )

    if (
        score_min < 0.0
        or
        score_max > 1.0
    ):

        class_scores = sigmoid(
            class_scores
        )


    # ========================================================
    # 类别
    # ========================================================

    class_ids = np.argmax(
        class_scores,
        axis=1
    )

    scores = np.max(
        class_scores,
        axis=1
    )


    # ========================================================
    # confidence
    # ========================================================

    valid = (
        scores
        >=
        CONF_THRESHOLD
    )

    boxes_xywh = boxes_xywh[
        valid
    ]

    class_ids = class_ids[
        valid
    ]

    scores = scores[
        valid
    ]

    if len(boxes_xywh) == 0:

        return []


    # ========================================================
    # xywh -> xyxy
    # ========================================================

    boxes = np.zeros_like(
        boxes_xywh,
        dtype=np.float32
    )

    boxes[:, 0] = (
        boxes_xywh[:, 0]
        -
        boxes_xywh[:, 2] / 2.0
    )

    boxes[:, 1] = (
        boxes_xywh[:, 1]
        -
        boxes_xywh[:, 3] / 2.0
    )

    boxes[:, 2] = (
        boxes_xywh[:, 0]
        +
        boxes_xywh[:, 2] / 2.0
    )

    boxes[:, 3] = (
        boxes_xywh[:, 1]
        +
        boxes_xywh[:, 3] / 2.0
    )


    detections = []


    # ========================================================
    # 分类别 NMS
    # ========================================================

    for cls_id in np.unique(
        class_ids
    ):

        indices = np.where(
            class_ids
            ==
            cls_id
        )[0]

        cls_boxes = boxes[
            indices
        ]

        cls_scores = scores[
            indices
        ]

        keep = nms_boxes(
            cls_boxes,
            cls_scores
        )

        for k in keep:

            detections.append(
                (
                    cls_boxes[k],
                    int(cls_id),
                    float(
                        cls_scores[k]
                    )
                )
            )


    detections.sort(
        key=lambda x: x[2],
        reverse=True
    )

    return detections[
        :MAX_DETECTIONS
    ]


# ============================================================
# LetterBox 坐标恢复
# ============================================================

def restore_box(
    box,
    scale,
    dx,
    dy,
    original_w,
    original_h
):

    x1, y1, x2, y2 = box

    x1 = (
        x1 - dx
    ) / scale

    y1 = (
        y1 - dy
    ) / scale

    x2 = (
        x2 - dx
    ) / scale

    y2 = (
        y2 - dy
    ) / scale

    x1 = int(
        np.clip(
            x1,
            0,
            original_w - 1
        )
    )

    y1 = int(
        np.clip(
            y1,
            0,
            original_h - 1
        )
    )

    x2 = int(
        np.clip(
            x2,
            0,
            original_w - 1
        )
    )

    y2 = int(
        np.clip(
            y2,
            0,
            original_h - 1
        )
    )

    return (
        x1,
        y1,
        x2,
        y2
    )


# ============================================================
# ZMQ初始化
# ============================================================

def init_zmq_publisher():

    context = zmq.Context()

    socket = context.socket(
        zmq.PUB
    )

    # --------------------------------------------------------
    # 只保留最新数据
    # 避免检测结果大量排队
    # --------------------------------------------------------

    socket.setsockopt(
        zmq.SNDHWM,
        1
    )

    socket.bind(
        ZMQ_PUB_ADDRESS
    )

    print()
    print("=" * 60)

    print(
        "检测结果 ZMQ 发布启动"
    )

    print(
        "地址:",
        ZMQ_PUB_ADDRESS
    )

    print("=" * 60)

    return (
        context,
        socket
    )


# ============================================================
# 发布当前检测结果
# ============================================================

def publish_detection_result(
    socket,
    frame_id,
    objects
):

    now = datetime.now()

    message = {

        "timestamp":
            now.strftime(
                "%Y-%m-%d %H:%M:%S.%f"
            )[:-3],

        "timestamp_ms":
            int(
                time.time() * 1000
            ),

        "frame_id":
            int(frame_id),

        "object_count":
            len(objects),

        "objects":
            objects
    }

    try:

        socket.send_json(
            message,
            flags=zmq.NOBLOCK
        )

    except zmq.Again:

        # -----------------------------------------------------
        # 接收端来不及时直接丢弃旧结果
        #
        # 检测程序不能因此阻塞
        # -----------------------------------------------------

        pass


# ============================================================
# RK3588 NPU初始化
# ============================================================

def load_npu():

    print()

    print(
        "=" * 60
    )

    print(
        "模型:",
        RKNN_MODEL
    )

    print(
        "类别数量:",
        len(CLASSES)
    )

    print()

    for i, name in enumerate(
        CLASSES
    ):

        print(
            f"{i}: "
            f"{name} "
            f"{CLASS_NAMES_CN[i]}"
        )

    print(
        "=" * 60
    )


    rknn = RKNNLite()


    print()
    print(
        "加载RKNN模型..."
    )


    ret = rknn.load_rknn(
        RKNN_MODEL
    )

    if ret != 0:

        raise RuntimeError(
            "RKNN模型加载失败"
        )


    print(
        "RKNN模型加载成功"
    )


    print(
        "初始化RK3588 NPU..."
    )


    ret = rknn.init_runtime(
        core_mask=
        RKNNLite.NPU_CORE_0_1_2
    )

    if ret != 0:

        raise RuntimeError(
            "NPU初始化失败"
        )


    print(
        "RK3588 NPU初始化成功"
    )

    return rknn


# ============================================================
# 主程序
# ============================================================

def main():

    # ========================================================
    # RKNN
    # ========================================================

    rknn = load_npu()


    # ========================================================
    # ZMQ
    # ========================================================

    zmq_context, zmq_pub = init_zmq_publisher()


    # ========================================================
    # RTSP
    # ========================================================

    url = build_rtsp_url()

    print()
    print(
        "RTSP:"
    )

    print(
        url
    )


    reader = LatestFrameReader(
        url
    )

    reader.start()


    last_id = 0

    warmed = False

    output_checked = False


    fps = 0.0

    fps_count = 0

    fps_start = time.time()


    try:

        while True:

            last_id, frame = (
                reader.read_latest(
                    last_id
                )
            )


            if frame is None:

                time.sleep(
                    0.002
                )

                continue


            original_h, original_w = (
                frame.shape[:2]
            )


            # =================================================
            # 预处理
            # =================================================

            input_img, scale, dx, dy = preprocess(
                frame
            )


            # =================================================
            # NPU预热
            # =================================================

            if not warmed:

                print()

                print(
                    "===== RKNN INPUT ====="
                )

                print(
                    "shape:",
                    input_img.shape
                )

                print(
                    "dtype:",
                    input_img.dtype
                )

                print(
                    "min:",
                    int(
                        input_img.min()
                    )
                )

                print(
                    "max:",
                    int(
                        input_img.max()
                    )
                )

                print(
                    "======================"
                )


                print(
                    "NPU预热..."
                )


                warm_output = rknn.inference(
                    inputs=[
                        input_img
                    ]
                )


                if warm_output is None:

                    raise RuntimeError(
                        "NPU预热失败"
                    )


                print(
                    "NPU预热完成"
                )

                warmed = True


            # =================================================
            # RKNN 推理
            # =================================================

            t0 = time.perf_counter()


            outputs = rknn.inference(
                inputs=[
                    input_img
                ]
            )


            npu_ms = (
                time.perf_counter()
                -
                t0
            ) * 1000.0


            if outputs is None:

                continue


            # =================================================
            # 第一次检查
            # =================================================

            debug_this_frame = False


            if not output_checked:

                print()

                print(
                    "========== RKNN OUTPUT =========="
                )

                print(
                    "输出数量:",
                    len(outputs)
                )


                for i, out in enumerate(
                    outputs
                ):

                    arr = np.asarray(
                        out
                    )

                    print(
                        f"output[{i}] "
                        f"shape={arr.shape} "
                        f"dtype={arr.dtype} "
                        f"min={float(arr.min()):.6f} "
                        f"max={float(arr.max()):.6f}"
                    )


                print(
                    "================================="
                )


                expected = (
                    4
                    +
                    len(CLASSES)
                )


                if len(outputs) == 1:

                    arr = np.asarray(
                        outputs[0]
                    )


                    if (
                        arr.ndim == 3
                        and
                        (
                            arr.shape[1] == expected
                            or
                            arr.shape[2] == expected
                        )
                    ):

                        print()

                        print(
                            "模型结构检查通过。"
                        )

                        print(
                            "检测类别数量:",
                            len(CLASSES)
                        )

                        print(
                            "输出channel:",
                            expected
                        )

                    else:

                        print(
                            "警告：模型输出结构不匹配"
                        )


                debug_this_frame = True

                output_checked = True


            # =================================================
            # 后处理
            # =================================================

            detections = post_process(
                outputs,
                debug=debug_this_frame
            )


            valid_count = 0

            # =================================================
            # 新增：
            #
            # 当前这一帧的结构化检测结果
            # =================================================

            current_objects = []


            # =================================================
            # 绘制
            # =================================================

            for (
                box,
                cls_id,
                score
            ) in detections:


                x1, y1, x2, y2 = restore_box(
                    box,
                    scale,
                    dx,
                    dy,
                    original_w,
                    original_h
                )


                if (
                    x2 <= x1
                    or
                    y2 <= y1
                ):

                    continue


                if (
                    x2 - x1 < 5
                    or
                    y2 - y1 < 5
                ):

                    continue


                valid_count += 1


                if (
                    0
                    <=
                    cls_id
                    <
                    len(CLASSES)
                ):

                    name = CLASSES[
                        cls_id
                    ]

                    name_cn = CLASS_NAMES_CN[
                        cls_id
                    ]

                else:

                    name = (
                        f"class_{cls_id}"
                    )

                    name_cn = name


                center_x = (
                    x1 + x2
                ) // 2

                center_y = (
                    y1 + y2
                ) // 2


                # =================================================
                # 新增：
                #
                # 结构化目标数据
                # =================================================

                current_objects.append(
                    {
                        "class_id":
                            int(cls_id),

                        "class_name":
                            name,

                        "class_name_cn":
                            name_cn,

                        "confidence":
                            round(
                                float(score),
                                4
                            ),

                        "bbox": [
                            int(x1),
                            int(y1),
                            int(x2),
                            int(y2)
                        ],

                        "center": [
                            int(center_x),
                            int(center_y)
                        ]
                    }
                )


                # =================================================
                # 原来的bbox显示
                # =================================================

                cv2.rectangle(
                    frame,
                    (
                        x1,
                        y1
                    ),
                    (
                        x2,
                        y2
                    ),
                    (
                        0,
                        255,
                        0
                    ),
                    2
                )


                cv2.putText(
                    frame,
                    f"{name} {score:.2f}",
                    (
                        x1,
                        max(
                            20,
                            y1 - 5
                        )
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (
                        0,
                        255,
                        0
                    ),
                    2
                )


                cv2.circle(
                    frame,
                    (
                        center_x,
                        center_y
                    ),
                    4,
                    (
                        0,
                        0,
                        255
                    ),
                    -1
                )


            # =================================================
            # 新增：
            #
            # 把当前检测结果通过ZMQ发送
            #
            # 即使没有识别到东西，也发送：
            #
            # object_count = 0
            # objects = []
            #
            # 这很重要
            # =================================================

            publish_detection_result(
                zmq_pub,
                last_id,
                current_objects
            )


            # =================================================
            # 图像中心
            # =================================================

            cv2.drawMarker(
                frame,
                (
                    original_w // 2,
                    original_h // 2
                ),
                (
                    255,
                    0,
                    0
                ),
                cv2.MARKER_CROSS,
                30,
                2
            )


            # =================================================
            # FPS
            # =================================================

            fps_count += 1

            now = time.time()

            elapsed = (
                now
                -
                fps_start
            )


            if elapsed >= 1.0:

                fps = (
                    fps_count
                    /
                    elapsed
                )

                fps_count = 0

                fps_start = now


            if npu_ms > 0:

                npu_fps = (
                    1000.0
                    /
                    npu_ms
                )

            else:

                npu_fps = 0.0


            # =================================================
            # 显示信息
            # =================================================

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (
                    20,
                    40
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (
                    0,
                    255,
                    0
                ),
                2
            )


            cv2.putText(
                frame,
                f"NPU: {npu_ms:.1f} ms",
                (
                    20,
                    80
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (
                    0,
                    255,
                    255
                ),
                2
            )


            cv2.putText(
                frame,
                f"NPU FPS: {npu_fps:.1f}",
                (
                    20,
                    120
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (
                    255,
                    255,
                    0
                ),
                2
            )


            cv2.putText(
                frame,
                f"Objects: {valid_count}",
                (
                    20,
                    160
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (
                    255,
                    255,
                    255
                ),
                2
            )


            # 新增一行，用于确认发送功能已经开启

            cv2.putText(
                frame,
                "ZMQ PUB: 127.0.0.1:5566",
                (
                    20,
                    200
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (
                    255,
                    255,
                    255
                ),
                2
            )


            if YOLO_DISPLAY:

                cv2.imshow(
                    "YOLOv8 RK3588 - 10 Classes",
                    frame
                )


                key = (
                    cv2.waitKey(1)
                    &
                    0xFF
                )


                if key == ord("q"):

                    break


    except KeyboardInterrupt:

        print()

        print(
            "收到 Ctrl+C，准备退出..."
        )


    finally:

        print()

        print(
            "释放资源..."
        )


        reader.stop()


        # =====================================================
        # ZMQ释放
        # =====================================================

        zmq_pub.close(
            linger=0
        )

        zmq_context.term()


        time.sleep(
            0.1
        )


        rknn.release()


        cv2.destroyAllWindows()


        print(
            "程序结束"
        )


if __name__ == "__main__":

    main()
