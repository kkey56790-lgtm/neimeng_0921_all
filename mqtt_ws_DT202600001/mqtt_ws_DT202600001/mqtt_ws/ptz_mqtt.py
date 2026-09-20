#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import threading

import rospy
import paho.mqtt.client as mqtt

from onvif import ONVIFCamera


# ============================================================
# ROS 初始化
# ============================================================

rospy.init_node(
    "ptz_mqtt_onvif"
)


# ============================================================
# 海康 ONVIF 参数
# 从 launch 文件读取
# ============================================================

CAMERA_IP = str(
    rospy.get_param(
        "~ip",
        "192.168.2.64"
    )
)

CAMERA_PORT = int(
    rospy.get_param(
        "~port",
        80
    )
)

CAMERA_USERNAME = str(
    rospy.get_param(
        "~username",
        "admin"
    )
)

CAMERA_PASSWORD = str(
    rospy.get_param(
        "~password",
        "okwy1688"
    )
)


# ============================================================
# MQTT 参数
# ============================================================

MQTT_HOST = str(
    rospy.get_param(
        "~mqtt_host",
        "222.187.130.102"
    )
)

MQTT_PORT = int(
    rospy.get_param(
        "~mqtt_port",
        1883
    )
)

MQTT_USERNAME = str(
    rospy.get_param(
        "~mqtt_username",
        "autocar"
    )
)

MQTT_PASSWORD = str(
    rospy.get_param(
        "~mqtt_password",
        "123456"
    )
)


# ============================================================
# 机器人唯一编码
# ============================================================

ROBOT_CODE = str(
    rospy.get_param(
        "~robot_code",
        "DT202600001"
    )
)


# ============================================================
# MQTT Topic
#
# 注意：
# 最新协议前缀为：
# thing/robot/{robotCode}/
# ============================================================

SERVICE_TOPIC = (
    f"thing/robot/{ROBOT_CODE}/services"
)

SERVICE_REPLY_TOPIC = (
    f"thing/robot/{ROBOT_CODE}/services_reply"
)


# ============================================================
# 海康 ONVIF 云台
# ============================================================

class HikPTZ:

    def __init__(self):

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "正在连接海康 ONVIF 云台..."
        )

        rospy.loginfo(
            "摄像头地址: %s:%d",
            CAMERA_IP,
            CAMERA_PORT
        )

        # ----------------------------------------------------
        # 连接海康摄像头
        # ----------------------------------------------------

        self.camera = ONVIFCamera(
            CAMERA_IP,
            CAMERA_PORT,
            CAMERA_USERNAME,
            CAMERA_PASSWORD
        )

        # ----------------------------------------------------
        # 创建 ONVIF Media Service
        # ----------------------------------------------------

        self.media_service = (
            self.camera.create_media_service()
        )

        # ----------------------------------------------------
        # 创建 ONVIF PTZ Service
        # ----------------------------------------------------

        self.ptz_service = (
            self.camera.create_ptz_service()
        )

        # ----------------------------------------------------
        # 获取 Profile
        # ----------------------------------------------------

        profiles = (
            self.media_service.GetProfiles()
        )

        self.profile_token = None

        for profile in profiles:

            rospy.loginfo(
                "发现 Profile: %s Token=%s",
                profile.Name,
                profile.token
            )

            if profile.PTZConfiguration is not None:

                self.profile_token = (
                    profile.token
                )

                rospy.loginfo(
                    "选择 PTZ Profile: %s",
                    profile.Name
                )

                break

        # ----------------------------------------------------
        # 如果没有找到带 PTZConfiguration 的 Profile
        # ----------------------------------------------------

        if self.profile_token is None:

            self.profile_token = (
                profiles[0].token
            )

            rospy.logwarn(
                "未找到 PTZConfiguration，使用第一个 Profile"
            )

        rospy.loginfo(
            "ONVIF 云台连接成功"
        )

        rospy.loginfo(
            "========================================"
        )


    # ========================================================
    # 云台连续运动
    # ========================================================

    def move(
        self,
        pan,
        tilt
    ):

        request = (
            self.ptz_service.create_type(
                "ContinuousMove"
            )
        )

        request.ProfileToken = (
            self.profile_token
        )

        request.Velocity = {

            "PanTilt": {

                "x": pan,

                "y": tilt

            },

            "Zoom": {

                "x": 0.0

            }

        }

        self.ptz_service.ContinuousMove(
            request
        )


    # ========================================================
    # 云台停止
    # ========================================================

    def stop(self):

        request = (
            self.ptz_service.create_type(
                "Stop"
            )
        )

        request.ProfileToken = (
            self.profile_token
        )

        request.PanTilt = True

        request.Zoom = True

        self.ptz_service.Stop(
            request
        )


# ============================================================
# MQTT 云台控制
# ============================================================

class MQTTPTZController:

    def __init__(self):

        # ----------------------------------------------------
        # 初始化 ONVIF 云台
        # ----------------------------------------------------

        self.ptz = HikPTZ()

        # 防止多个 MQTT 指令同时调用云台
        self.ptz_lock = (
            threading.Lock()
        )


        # ----------------------------------------------------
        # 创建 MQTT Client
        #
        # 兼容 paho-mqtt 1.x / 2.x
        # ----------------------------------------------------

        try:

            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=f"{ROBOT_CODE}_ptz",
                protocol=mqtt.MQTTv311
            )

            rospy.loginfo(
                "使用 paho-mqtt Callback API VERSION2"
            )

        except (AttributeError, TypeError):

            self.client = mqtt.Client(
                client_id=f"{ROBOT_CODE}_ptz",
                protocol=mqtt.MQTTv311
            )

            rospy.loginfo(
                "使用旧版 paho-mqtt Callback API"
            )


        # ----------------------------------------------------
        # 设置 MQTT 用户名密码
        # ----------------------------------------------------

        self.client.username_pw_set(
            username=MQTT_USERNAME,
            password=MQTT_PASSWORD
        )


        # ----------------------------------------------------
        # 自动重连
        # ----------------------------------------------------

        self.client.reconnect_delay_set(
            min_delay=1,
            max_delay=10
        )


        # ----------------------------------------------------
        # MQTT 回调
        # ----------------------------------------------------

        self.client.on_connect = (
            self.on_connect
        )

        self.client.on_disconnect = (
            self.on_disconnect
        )

        self.client.on_message = (
            self.on_message
        )


    # ========================================================
    # 连接 MQTT
    # ========================================================

    def connect_mqtt(self):

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "正在连接 MQTT Broker..."
        )

        rospy.loginfo(
            "MQTT Broker: %s:%d",
            MQTT_HOST,
            MQTT_PORT
        )

        rospy.loginfo(
            "MQTT Username: %s",
            MQTT_USERNAME
        )

        rospy.loginfo(
            "robotCode: %s",
            ROBOT_CODE
        )

        rospy.loginfo(
            "控制 Topic: %s",
            SERVICE_TOPIC
        )

        rospy.loginfo(
            "回执 Topic: %s",
            SERVICE_REPLY_TOPIC
        )

        rospy.loginfo(
            "========================================"
        )

        self.client.connect(
            MQTT_HOST,
            MQTT_PORT,
            keepalive=60
        )

        # MQTT 网络循环后台线程
        self.client.loop_start()


    # ========================================================
    # MQTT 连接成功
    # ========================================================

    def on_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties=None
    ):

        try:

            code = int(
                reason_code
            )

        except Exception:

            code = reason_code


        if code == 0:

            rospy.loginfo(
                "========================================"
            )

            rospy.loginfo(
                "MQTT Broker 连接成功"
            )

            result, mid = (
                client.subscribe(
                    SERVICE_TOPIC,
                    qos=1
                )
            )

            rospy.loginfo(
                "已订阅 Topic:"
            )

            rospy.loginfo(
                "%s",
                SERVICE_TOPIC
            )

            rospy.loginfo(
                "subscribe result=%s mid=%s",
                result,
                mid
            )

            rospy.loginfo(
                "等待中台云台控制指令..."
            )

            rospy.loginfo(
                "========================================"
            )

        else:

            rospy.logerr(
                "MQTT 连接失败: %s",
                reason_code
            )


    # ========================================================
    # MQTT 断开
    # ========================================================

    def on_disconnect(
        self,
        client,
        userdata,
        *args
    ):

        rospy.logwarn(
            "MQTT Broker 连接断开"
        )


    # ========================================================
    # 发送指令执行回执
    #
    # 协议：
    #
    # thing/robot/{robotCode}/services_reply
    #
    # {
    #   "tid": "...",
    #   "method": "ptz_direction_control",
    #   "timestamp": ...,
    #   "data": {
    #       "result": 0
    #   }
    # }
    #
    # result:
    #   0   成功
    #  -1   执行失败
    #  404  method 不存在
    # ========================================================

    def send_reply(
        self,
        tid,
        method,
        result
    ):

        message = {

            "tid": tid,

            "method": method,

            "timestamp": int(
                time.time() * 1000
            ),

            "data": {

                "result": result

            }

        }

        payload = json.dumps(
            message,
            ensure_ascii=False
        )

        info = self.client.publish(
            SERVICE_REPLY_TOPIC,
            payload,
            qos=1
        )

        rospy.loginfo(
            "---------- MQTT 回执 ----------"
        )

        rospy.loginfo(
            "Topic: %s",
            SERVICE_REPLY_TOPIC
        )

        rospy.loginfo(
            "Payload: %s",
            payload
        )

        rospy.loginfo(
            "publish rc=%s",
            info.rc
        )


    # ========================================================
    # 收到 MQTT 控制消息
    # ========================================================

    def on_message(
        self,
        client,
        userdata,
        msg
    ):

        rospy.loginfo(
            ""
        )

        rospy.loginfo(
            "========================================"
        )

        rospy.loginfo(
            "收到 MQTT 控制指令"
        )

        rospy.loginfo(
            "Topic: %s",
            msg.topic
        )


        # ----------------------------------------------------
        # bytes -> string
        # ----------------------------------------------------

        try:

            payload_text = (
                msg.payload.decode(
                    "utf-8"
                )
            )

        except Exception as e:

            rospy.logerr(
                "MQTT Payload 解码失败: %s",
                e
            )

            return


        rospy.loginfo(
            "Payload: %s",
            payload_text
        )


        # ----------------------------------------------------
        # JSON
        # ----------------------------------------------------

        try:

            message = json.loads(
                payload_text
            )

        except Exception as e:

            rospy.logerr(
                "JSON 解析失败: %s",
                e
            )

            return


        # ----------------------------------------------------
        # Envelope
        # ----------------------------------------------------

        tid = str(
            message.get(
                "tid",
                ""
            )
        )

        method = str(
            message.get(
                "method",
                ""
            )
        )

        data = message.get(
            "data",
            {}
        )


        # ----------------------------------------------------
        # 参数合法性
        # ----------------------------------------------------

        if not tid:

            rospy.logerr(
                "MQTT 指令缺少 tid"
            )

            return


        if not method:

            rospy.logerr(
                "MQTT 指令缺少 method"
            )

            self.send_reply(
                tid,
                "",
                -1
            )

            return


        if not isinstance(
            data,
            dict
        ):

            rospy.logerr(
                "data 必须是 JSON Object"
            )

            self.send_reply(
                tid,
                method,
                -1
            )

            return


        # ----------------------------------------------------
        # Method 分发
        # ----------------------------------------------------

        try:

            if method == "ptz_direction_control":

                self.handle_ptz_direction(
                    data
                )

                self.send_reply(
                    tid,
                    method,
                    0
                )

            else:

                rospy.logwarn(
                    "不支持的 method: %s",
                    method
                )

                self.send_reply(
                    tid,
                    method,
                    404
                )


        except Exception as e:

            rospy.logerr(
                "PTZ 指令执行失败: %s",
                e
            )

            self.send_reply(
                tid,
                method,
                -1
            )


    # ========================================================
    # 处理云台方向控制
    #
    # MQTT:
    #
    # {
    #   "direction":"left",
    #   "start":true,
    #   "speed":5
    # }
    #
    # ========================================================

    def handle_ptz_direction(
        self,
        data
    ):

        # ----------------------------------------------------
        # direction
        # ----------------------------------------------------

        direction = str(
            data.get(
                "direction",
                ""
            )
        ).strip().lower()


        # ----------------------------------------------------
        # start
        # ----------------------------------------------------

        start = data.get(
            "start",
            False
        )


        # ----------------------------------------------------
        # 兼容字符串形式 true / false
        # ----------------------------------------------------

        if isinstance(
            start,
            str
        ):

            start = (
                start.lower()
                in [
                    "true",
                    "1",
                    "yes"
                ]
            )


        # ----------------------------------------------------
        # speed
        # ----------------------------------------------------

        speed_level = data.get(
            "speed",
            5
        )


        try:

            speed_level = float(
                speed_level
            )

        except Exception:

            raise ValueError(
                "speed 必须为数字"
            )


        # ----------------------------------------------------
        # MQTT 协议限制 1 ~ 10
        # ----------------------------------------------------

        if speed_level < 1:

            speed_level = 1

        if speed_level > 10:

            speed_level = 10


        # ----------------------------------------------------
        # MQTT 速度 1~10
        #
        # 转换 ONVIF 速度：
        #
        # 1  -> 0.1
        # 5  -> 0.5
        # 10 -> 1.0
        # ----------------------------------------------------

        speed = (
            speed_level / 10.0
        )


        rospy.loginfo(
            "PTZ参数 direction=%s start=%s speed=%s -> ONVIF %.2f",
            direction,
            start,
            speed_level,
            speed
        )


        # ====================================================
        # start = false
        #
        # 不管 direction 是什么
        # 直接停止云台
        # ====================================================

        if not start:

            with self.ptz_lock:

                self.ptz.stop()

            rospy.loginfo(
                ">>> PTZ STOP"
            )

            return


        # ====================================================
        # start = true
        # ====================================================

        with self.ptz_lock:


            # ------------------------------------------------
            # 上
            # ------------------------------------------------

            if direction == "up":

                rospy.loginfo(
                    ">>> PTZ UP"
                )

                self.ptz.move(
                    0.0,
                    speed
                )


            # ------------------------------------------------
            # 下
            # ------------------------------------------------

            elif direction == "down":

                rospy.loginfo(
                    ">>> PTZ DOWN"
                )

                self.ptz.move(
                    0.0,
                    -speed
                )


            # ------------------------------------------------
            # 左
            # ------------------------------------------------

            elif direction == "left":

                rospy.loginfo(
                    ">>> PTZ LEFT"
                )

                self.ptz.move(
                    -speed,
                    0.0
                )


            # ------------------------------------------------
            # 右
            # ------------------------------------------------

            elif direction == "right":

                rospy.loginfo(
                    ">>> PTZ RIGHT"
                )

                self.ptz.move(
                    speed,
                    0.0
                )


            else:

                raise ValueError(
                    "未知 direction: "
                    + direction
                )


    # ========================================================
    # ROS 主循环
    # ========================================================

    def run(self):

        # ----------------------------------------------------
        # MQTT连接
        # ----------------------------------------------------

        self.connect_mqtt()

        rospy.loginfo(
            "MQTT + ONVIF 云台控制程序运行中"
        )

        try:

            rospy.spin()

        finally:

            # ------------------------------------------------
            # 程序退出时停止云台
            # ------------------------------------------------

            rospy.loginfo(
                "程序退出，停止云台..."
            )

            try:

                with self.ptz_lock:

                    self.ptz.stop()

            except Exception as e:

                rospy.logwarn(
                    "停止云台失败: %s",
                    e
                )


            # ------------------------------------------------
            # MQTT退出
            # ------------------------------------------------

            try:

                self.client.loop_stop()

                self.client.disconnect()

            except Exception:

                pass


# ============================================================
# main
# ============================================================

if __name__ == "__main__":

    try:

        controller = (
            MQTTPTZController()
        )

        controller.run()

    except rospy.ROSInterruptException:

        pass

    except Exception as e:

        rospy.logerr(
            "程序启动失败: %s",
            e
        )
