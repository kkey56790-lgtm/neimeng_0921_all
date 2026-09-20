#!/usr/bin/env python3
"""Read-only validation of remote robot WAV files through HTTP V2.4."""

import argparse
import io
import sys
import wave

import requests


VOICE_FILES = {
    "01_inspecting.wav": "正在巡检中，请勿靠近",
    "02_stopped.wav": "机器人停车，请避让",
    "03_reversing.wav": "倒车请注意",
    "04_turn_left.wav": "左转",
    "05_turn_right.wav": "右转",
    "06_avoiding_obstacle.wav": "机器人避障中",
}


def require_success(response, operation):
    response.raise_for_status()
    payload = response.json()
    if not isinstance(payload, dict) or not payload.get("result", False):
        detail = payload.get("data", payload) if isinstance(payload, dict) else payload
        raise RuntimeError("{} 失败: {}".format(operation, detail))
    return payload


def inspect_wav(content):
    try:
        with wave.open(io.BytesIO(content), "rb") as wav_file:
            channels = wav_file.getnchannels()
            width = wav_file.getsampwidth()
            rate = wav_file.getframerate()
            frames = wav_file.getnframes()
            compression = wav_file.getcomptype()
    except (EOFError, wave.Error) as exc:
        return False, "WAV 无法解析: {}".format(exc)
    duration = frames / float(rate) if rate else 0.0
    valid = (channels in (1, 2) and width == 2 and rate >= 8000
             and frames > 0 and compression == "NONE")
    return valid, "{}声道 {}位 {}Hz {:.2f}秒".format(channels, width * 8, rate, duration)


def main():
    parser = argparse.ArgumentParser(description="通过 HTTP V2.4 只读检查机器人六个语音文件。")
    parser.add_argument("--robot-ip", default="192.168.2.100")
    parser.add_argument("--port", type=int, default=7999)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--download-check", action="store_true",
                        help="下载到内存并验证 WAV 格式")
    args = parser.parse_args()
    base_url = "http://{}:{}".format(args.robot_ip, args.port)
    session = requests.Session()
    try:
        response = session.get(base_url + "/system_manager/get_wav_list",
                               timeout=args.timeout)
        data = require_success(response, "获取语音列表").get("data", [])
        listed = {str(item.get("name")): item for item in data if isinstance(item, dict)}
    except Exception as exc:
        print("检查失败: {}".format(exc), file=sys.stderr)
        return 1

    errors = []
    print("机器人语音列表:")
    for filename, phrase in VOICE_FILES.items():
        item = listed.get(filename)
        if not item:
            print("[缺失] {} - {}".format(filename, phrase))
            errors.append(filename)
            continue
        print("[存在] {} - {} - {} B".format(filename, phrase, item.get("size", "未知")))
        if args.download_check:
            try:
                response = session.post(
                    base_url + "/system_manager/download_wav",
                    params={"file_name": filename}, timeout=args.timeout)
                response.raise_for_status()
                valid, details = inspect_wav(response.content)
                print("       [{}] {}".format("格式通过" if valid else "格式失败", details))
                if not valid:
                    errors.append(filename + " 格式")
            except Exception as exc:
                print("       [下载失败] {}".format(exc))
                errors.append(filename + " 下载")
    extras = sorted(name for name in listed if name not in VOICE_FILES)
    if extras:
        print("其他服务器语音: {}".format(", ".join(extras)))
    if errors:
        print("检查结果: 失败（{} 项）".format(len(errors)))
        return 1
    print("检查结果: 通过，六个远程语音文件均存在。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
