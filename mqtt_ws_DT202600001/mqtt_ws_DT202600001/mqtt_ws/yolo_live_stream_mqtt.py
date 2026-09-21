#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 实时检测画面 + MQTT + RTMP 推流

功能：
1. 从 RTSP 摄像头读取视频
2. 从 ZMQ 接收 YOLO 检测结果
3. 在视频上绘制检测框/类别/置信度
4. 使用 FFmpeg libx264 软件编码
5. 推流到 RTMP
6. 连接 MQTT 中台
7. FFmpeg 异常退出后自动重启
8. 避免 FFmpeg stdin BrokenPipe 导致程序崩溃

Ubuntu 20.04 / Python3
"""

import cv2
import zmq
import json
import time
import queue
import signal
import threading
import subprocess
import sys
import os

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] 缺少 paho-mqtt")
    print("安装：pip3 install paho-mqtt")
    sys.exit(1)


# ============================================================
# 1. 基本配置
# ============================================================

# --------------------------
# 机器人
# --------------------------
ROBOT_CODE = "DT202600001"
ROBOT_NAME = "内蒙古巡检车-DT202600001"


# --------------------------
# YOLO ZMQ
# --------------------------
YOLO_ZMQ_URL = "tcp://127.0.0.1:5566"

# 0919-2.rknn：类别顺序来自其源模型 0919.onnx 的 names 元数据。
# 此程序接收检测 JSON，不负责加载 RKNN 或执行推理。
DETECTION_CLASSES = (
    ("chocks", "挡掩"),
    ("extinguisher", "灭火器"),
    ("plate", "车牌"),
    ("tag", "检修牌"),
    ("light", "车灯"),
    ("support", "支护"),
    ("screw", "轮毂螺丝"),
    ("tank", "油箱"),
    ("lamp_broken", "车灯破损"),
    ("box_broken", "箱体破损"),
    ("warning", "警告标志"),
)


def normalize_detection(data):
    """按新模型 ID 统一 MQTT 和直播标签，保留坐标、置信度等字段。"""
    result = dict(data)
    objects = data.get("objects", [])
    if not isinstance(objects, list):
        objects = []
    normalized = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        item = dict(obj)
        raw_id = item.get("class_id")
        class_id = None
        if isinstance(raw_id, int) and not isinstance(raw_id, bool):
            class_id = raw_id
        elif isinstance(raw_id, str) and raw_id.strip().isdigit():
            class_id = int(raw_id.strip())
        if class_id is not None and 0 <= class_id < len(DETECTION_CLASSES):
            item["class_id"] = class_id
            item["class_name"], item["class_name_cn"] = DETECTION_CLASSES[class_id]
        else:
            # 不把未知 ID 误标为某个检测项目。
            item["class_name"] = "unknown_{}".format(raw_id)
            item["class_name_cn"] = "未知类别({})".format(raw_id)
        normalized.append(item)
    result["objects"] = normalized
    result["object_count"] = len(normalized)
    return result


# --------------------------
# 摄像头 RTSP
# --------------------------
#
# !!! 这里改成你当前已经能正常连接的 RTSP 地址 !!!
#
# 示例：
# RTSP_URL = "rtsp://admin:password@192.168.2.64:554/Streaming/Channels/102"
#
RTSP_URL = "rtsp://admin:okwy1688@192.168.2.64:554/Streaming/Channels/102"


# --------------------------
# 输出视频参数
# --------------------------
OUTPUT_WIDTH = 1280
OUTPUT_HEIGHT = 720
OUTPUT_FPS = 15


# --------------------------
# RTMP
# --------------------------
RTMP_URL = (
    "rtmp://222.187.130.102:1935/live/"
    + ROBOT_CODE
)


# --------------------------
# MQTT
# --------------------------
MQTT_HOST = "222.187.130.102"
MQTT_PORT = 1883

MQTT_USERNAME = None
MQTT_PASSWORD = None

MQTT_KEEPALIVE = 60

MQTT_SERVICE_TOPIC = (
    f"thing/robot/{ROBOT_CODE}/services"
)

MQTT_REPLY_TOPIC = (
    f"thing/robot/{ROBOT_CODE}/services_reply"
)


# ============================================================
# 2. 全局变量
# ============================================================

running = True

latest_detection = {
    "timestamp": 0,
    "frame_id": 0,
    "object_count": 0,
    "objects": []
}

detection_lock = threading.Lock()

ffmpeg_process = None
ffmpeg_lock = threading.Lock()

mqtt_client = None


# ============================================================
# 3. 信号处理
# ============================================================

def signal_handler(sig, frame):
    global running

    print("\n[STOP] 收到退出信号")

    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# ============================================================
# 4. YOLO ZMQ 接收线程
# ============================================================

def yolo_receiver():
    global latest_detection
    global running

    context = zmq.Context()

    socket = context.socket(zmq.SUB)

    socket.setsockopt_string(
        zmq.SUBSCRIBE,
        ""
    )

    socket.setsockopt(
        zmq.RCVTIMEO,
        1000
    )

    socket.setsockopt(
        zmq.LINGER,
        0
    )

    try:
        socket.connect(YOLO_ZMQ_URL)

        print(
            f"[YOLO] detection input: "
            f"{YOLO_ZMQ_URL}"
        )

    except Exception as e:

        print(
            f"[YOLO] ZMQ连接失败: {e}"
        )

        return

    while running:

        try:

            message = socket.recv()

            try:

                text = message.decode(
                    "utf-8"
                )

                data = json.loads(text)

            except Exception:

                try:

                    data = json.loads(
                        message
                    )

                except Exception:

                    continue

            if not isinstance(
                data,
                dict
            ):
                continue

            with detection_lock:

                latest_detection = normalize_detection(data)

        except zmq.Again:

            continue

        except Exception as e:

            if running:

                print(
                    f"[YOLO] 接收异常: {e}"
                )

                time.sleep(0.2)

    try:
        socket.close()
    except Exception:
        pass

    try:
        context.term()
    except Exception:
        pass


# ============================================================
# 5. MQTT
# ============================================================

def mqtt_on_connect(
    client,
    userdata,
    flags,
    rc
):

    if rc == 0:

        print(
            f"[MQTT] connected: "
            f"tcp://{MQTT_HOST}:{MQTT_PORT}"
        )

        client.subscribe(
            MQTT_SERVICE_TOPIC
        )

        print(
            f"[MQTT] subscribe: "
            f"{MQTT_SERVICE_TOPIC}"
        )

    else:

        print(
            f"[MQTT] connect failed "
            f"rc={rc}"
        )


def mqtt_on_disconnect(
    client,
    userdata,
    rc
):

    if running:

        print(
            f"[MQTT] disconnected "
            f"rc={rc}"
        )


def mqtt_on_message(
    client,
    userdata,
    msg
):

    try:

        payload = msg.payload.decode(
            "utf-8",
            errors="ignore"
        )

        print(
            f"[MQTT] RX topic={msg.topic}"
        )

        print(
            f"[MQTT] RX payload={payload}"
        )

        try:

            data = json.loads(payload)

        except Exception:

            return

        handle_mqtt_command(
            client,
            data
        )

    except Exception as e:

        print(
            f"[MQTT] message error: {e}"
        )


def handle_mqtt_command(
    client,
    data
):

    """
    这里可以继续加入中台控制命令。

    当前支持：
        get_detection
        get_stream
    """

    method = (
        data.get("method")
        or data.get("service")
        or data.get("cmd")
        or ""
    )

    method = str(
        method
    ).strip()

    # ----------------------------------------
    # 查询当前检测结果
    # ----------------------------------------

    if method == "get_detection":

        with detection_lock:

            det = dict(
                latest_detection
            )

        reply = {
            "robotCode": ROBOT_CODE,
            "method": method,
            "result": 0,
            "data": det
        }

        mqtt_publish_json(
            MQTT_REPLY_TOPIC,
            reply
        )

    # ----------------------------------------
    # 查询推流地址
    # ----------------------------------------

    elif method == "get_stream":

        reply = {
            "robotCode": ROBOT_CODE,
            "method": method,
            "result": 0,
            "data": {
                "rtmp": RTMP_URL
            }
        }

        mqtt_publish_json(
            MQTT_REPLY_TOPIC,
            reply
        )


def mqtt_publish_json(
    topic,
    data
):

    global mqtt_client

    if mqtt_client is None:
        return

    try:

        payload = json.dumps(
            data,
            ensure_ascii=False
        )

        mqtt_client.publish(
            topic,
            payload
        )

    except Exception as e:

        print(
            f"[MQTT] publish error: {e}"
        )


def start_mqtt():

    global mqtt_client

    try:

        mqtt_client = mqtt.Client()

        mqtt_client.on_connect = (
            mqtt_on_connect
        )

        mqtt_client.on_disconnect = (
            mqtt_on_disconnect
        )

        mqtt_client.on_message = (
            mqtt_on_message
        )

        if (
            MQTT_USERNAME is not None
            and
            MQTT_PASSWORD is not None
        ):

            mqtt_client.username_pw_set(
                MQTT_USERNAME,
                MQTT_PASSWORD
            )

        mqtt_client.connect_async(
            MQTT_HOST,
            MQTT_PORT,
            MQTT_KEEPALIVE
        )

        mqtt_client.loop_start()

    except Exception as e:

        print(
            f"[MQTT] start error: {e}"
        )


# ============================================================
# 6. FFmpeg
# ============================================================

def build_ffmpeg_command():

    """
    使用 libx264 软件编码。

    不再使用：
        h264_v4l2m2m

    防止：
        Could not find a valid device
    """

    command = [

        "ffmpeg",

        "-hide_banner",

        "-loglevel",
        "warning",

        # ---------------------------------
        # 输入：Python OpenCV BGR raw video
        # ---------------------------------

        "-f",
        "rawvideo",

        "-vcodec",
        "rawvideo",

        "-pix_fmt",
        "bgr24",

        "-s",
        f"{OUTPUT_WIDTH}x{OUTPUT_HEIGHT}",

        "-r",
        str(OUTPUT_FPS),

        "-i",
        "-",

        # ---------------------------------
        # 不使用音频
        # ---------------------------------

        "-an",

        # ---------------------------------
        # libx264 软件编码
        # ---------------------------------

        "-c:v",
        "libx264",

        # CPU优先速度
        "-preset",
        "ultrafast",

        # 实时推流低延迟
        "-tune",
        "zerolatency",

        # ---------------------------------
        # H264格式
        # ---------------------------------

        "-pix_fmt",
        "yuv420p",

        # ---------------------------------
        # 码率
        # ---------------------------------

        "-b:v",
        "2500k",

        "-maxrate",
        "2500k",

        "-bufsize",
        "5000k",

        # ---------------------------------
        # GOP
        # 15fps × 2秒 = 30
        # ---------------------------------

        "-g",
        str(
            OUTPUT_FPS * 2
        ),

        "-keyint_min",
        str(
            OUTPUT_FPS
        ),

        "-sc_threshold",
        "0",

        # ---------------------------------
        # FLV → RTMP
        # ---------------------------------

        "-f",
        "flv",

        RTMP_URL
    ]

    return command


def ffmpeg_log_reader(
    process
):

    try:

        while running:

            if (
                process is None
                or
                process.stderr is None
            ):
                break

            line = (
                process.stderr.readline()
            )

            if not line:

                if (
                    process.poll()
                    is not None
                ):
                    break

                continue

            text = line.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if text:

                print(
                    f"[FFMPEG] {text}"
                )

    except Exception:
        pass


def start_ffmpeg():

    global ffmpeg_process

    with ffmpeg_lock:

        stop_ffmpeg()

        command = (
            build_ffmpeg_command()
        )

        print(
            f"[FFMPEG] encoder=libx264 "
            f"output={RTMP_URL}"
        )

        try:

            ffmpeg_process = (
                subprocess.Popen(

                    command,

                    stdin=subprocess.PIPE,

                    stdout=subprocess.DEVNULL,

                    stderr=subprocess.PIPE,

                    bufsize=0
                )
            )

            thread = threading.Thread(
                target=ffmpeg_log_reader,
                args=(
                    ffmpeg_process,
                ),
                daemon=True
            )

            thread.start()

            # 给 FFmpeg 一点启动时间
            time.sleep(0.5)

            if (
                ffmpeg_process.poll()
                is not None
            ):

                print(
                    "[FFMPEG] 启动失败"
                )

                return False

            return True

        except Exception as e:

            print(
                f"[FFMPEG] start error: {e}"
            )

            ffmpeg_process = None

            return False


def stop_ffmpeg():

    global ffmpeg_process

    if ffmpeg_process is None:
        return

    try:

        if (
            ffmpeg_process.stdin
            is not None
        ):

            try:

                ffmpeg_process.stdin.close()

            except Exception:

                pass

        if (
            ffmpeg_process.poll()
            is None
        ):

            ffmpeg_process.terminate()

            try:

                ffmpeg_process.wait(
                    timeout=2
                )

            except Exception:

                try:

                    ffmpeg_process.kill()

                except Exception:

                    pass

    except Exception:

        pass

    ffmpeg_process = None


def write_frame_to_ffmpeg(
    frame
):

    global ffmpeg_process

    # FFmpeg不存在
    if ffmpeg_process is None:

        return False

    # FFmpeg已经退出
    if (
        ffmpeg_process.poll()
        is not None
    ):

        return False

    if (
        ffmpeg_process.stdin
        is None
    ):

        return False

    try:

        ffmpeg_process.stdin.write(
            frame.tobytes()
        )

        return True

    except BrokenPipeError:

        print(
            "[FFMPEG] BrokenPipe，"
            "FFmpeg已经退出"
        )

        return False

    except OSError as e:

        print(
            f"[FFMPEG] stdin error: {e}"
        )

        return False

    except Exception as e:

        print(
            f"[FFMPEG] write error: {e}"
        )

        return False


# ============================================================
# 7. 绘制 YOLO 检测结果
# ============================================================

def draw_detection(
    frame
):

    with detection_lock:

        data = latest_detection

        if not isinstance(
            data,
            dict
        ):
            return frame

        objects = data.get(
            "objects",
            []
        )

    if not isinstance(
        objects,
        list
    ):
        return frame

    for obj in objects:

        try:

            bbox = obj.get(
                "bbox",
                []
            )

            if (
                not isinstance(
                    bbox,
                    list
                )
                or
                len(bbox) < 4
            ):
                continue

            x1 = int(
                bbox[0]
            )

            y1 = int(
                bbox[1]
            )

            x2 = int(
                bbox[2]
            )

            y2 = int(
                bbox[3]
            )

            class_name = (
                obj.get(
                    "class_name"
                )
                or
                str(
                    obj.get(
                        "class_id",
                        ""
                    )
                )
            )

            confidence = float(
                obj.get(
                    "confidence",
                    0
                )
            )

            # ------------------------
            # 检测框
            # ------------------------

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            text = (
                f"{class_name} "
                f"{confidence:.2f}"
            )

            # OpenCV 默认字体不支持中文；直播用英文，MQTT 保留中英文。

            cv2.putText(
                frame,
                text,
                (
                    x1,
                    max(
                        20,
                        y1 - 8
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        except Exception:

            continue

    return frame


# ============================================================
# 8. 摄像头
# ============================================================

def open_camera():

    print(
        "[RTSP] connecting..."
    )

    # 优先使用 FFmpeg backend
    cap = cv2.VideoCapture(
        RTSP_URL,
        cv2.CAP_FFMPEG
    )

    # 减少缓存
    try:

        cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

    except Exception:

        pass

    if not cap.isOpened():

        print(
            "[RTSP] camera connect failed"
        )

        cap.release()

        return None

    print(
        "[RTSP] camera connected"
    )

    return cap


# ============================================================
# 9. 主程序
# ============================================================

def main():

    global running

    print("=" * 70)

    print(
        "YOLO LIVE STREAM MQTT"
    )

    print("=" * 70)

    # --------------------------------
    # 基本信息
    # --------------------------------

    print(
        f"[ROBOT] "
        f"{ROBOT_CODE} "
        f"{ROBOT_NAME}"
    )

    print(
        f"[VIDEO] annotated output: "
        f"{OUTPUT_WIDTH}x{OUTPUT_HEIGHT} "
        f"@ {OUTPUT_FPS} fps"
    )

    # --------------------------------
    # YOLO ZMQ线程
    # --------------------------------

    yolo_thread = threading.Thread(
        target=yolo_receiver,
        daemon=True
    )

    yolo_thread.start()

    # --------------------------------
    # MQTT
    # --------------------------------

    start_mqtt()

    # --------------------------------
    # RTSP
    # --------------------------------

    cap = None

    while (
        running
        and
        cap is None
    ):

        cap = open_camera()

        if cap is None:

            print(
                "[RTSP] 3秒后重新连接"
            )

            time.sleep(3)

    if not running:

        return

    # --------------------------------
    # FFmpeg
    # --------------------------------

    ffmpeg_ok = (
        start_ffmpeg()
    )

    if not ffmpeg_ok:

        print(
            "[FFMPEG] 首次启动失败"
        )

    # --------------------------------
    # FPS控制
    # --------------------------------

    frame_interval = (
        1.0
        /
        float(
            OUTPUT_FPS
        )
    )

    next_frame_time = time.time()

    camera_fail_count = 0

    ffmpeg_restart_time = 0

    # ========================================================
    # 主循环
    # ========================================================

    while running:

        # --------------------------------
        # 摄像头掉线
        # --------------------------------

        if (
            cap is None
            or
            not cap.isOpened()
        ):

            try:

                if cap is not None:

                    cap.release()

            except Exception:

                pass

            cap = None

            print(
                "[RTSP] reconnecting..."
            )

            time.sleep(2)

            cap = open_camera()

            continue

        # --------------------------------
        # 读取帧
        # --------------------------------

        ret, frame = cap.read()

        if (
            not ret
            or
            frame is None
        ):

            camera_fail_count += 1

            if (
                camera_fail_count
                >= 30
            ):

                print(
                    "[RTSP] 连续读取失败，"
                    "重新连接摄像头"
                )

                try:

                    cap.release()

                except Exception:

                    pass

                cap = None

                camera_fail_count = 0

            time.sleep(0.02)

            continue

        camera_fail_count = 0

        # --------------------------------
        # FPS限制
        # --------------------------------

        now = time.time()

        if now < next_frame_time:

            time.sleep(
                next_frame_time
                -
                now
            )

        next_frame_time = (
            time.time()
            +
            frame_interval
        )

        # --------------------------------
        # resize
        # --------------------------------

        if (
            frame.shape[1]
            != OUTPUT_WIDTH
            or
            frame.shape[0]
            != OUTPUT_HEIGHT
        ):

            frame = cv2.resize(
                frame,
                (
                    OUTPUT_WIDTH,
                    OUTPUT_HEIGHT
                )
            )

        # --------------------------------
        # YOLO绘制
        # --------------------------------

        frame = draw_detection(
            frame
        )

        # --------------------------------
        # FFmpeg状态检查
        # --------------------------------

        ffmpeg_alive = (

            ffmpeg_process
            is not None

            and

            ffmpeg_process.poll()
            is None
        )

        if not ffmpeg_alive:

            current_time = time.time()

            # 最多每3秒重启一次
            if (
                current_time
                -
                ffmpeg_restart_time
                >= 3
            ):

                ffmpeg_restart_time = (
                    current_time
                )

                print(
                    "[FFMPEG] encoder stopped, "
                    "restarting..."
                )

                start_ffmpeg()

            continue

        # --------------------------------
        # 推流
        # --------------------------------

        ok = write_frame_to_ffmpeg(
            frame
        )

        if not ok:

            print(
                "[FFMPEG] frame write failed"
            )

            stop_ffmpeg()

            continue


    # ========================================================
    # 退出
    # ========================================================

    print(
        "[STOP] 正在关闭..."
    )

    try:

        if cap is not None:

            cap.release()

    except Exception:

        pass

    stop_ffmpeg()

    try:

        if mqtt_client is not None:

            mqtt_client.loop_stop()

            mqtt_client.disconnect()

    except Exception:

        pass

    cv2.destroyAllWindows()

    print(
        "[STOP] 程序已退出"
    )


# ============================================================
# 10. 启动
# ============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        running = False

        print(
            "\n[STOP] KeyboardInterrupt"
        )

    except Exception as e:

        print(
            f"[FATAL] {e}"
        )

        import traceback

        traceback.print_exc()

    finally:

        try:

            stop_ffmpeg()

        except Exception:

            pass
