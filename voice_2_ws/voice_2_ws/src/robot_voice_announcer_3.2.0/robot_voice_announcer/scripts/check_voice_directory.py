#!/usr/bin/env python3
"""Check the robot voice directory without changing any files."""

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import wave
from pathlib import Path


VOICE_FILES = {
    "01_inspecting.wav": "正在巡检中，请勿靠近",
    "02_stopped.wav": "机器人停车，请避让",
    "03_reversing.wav": "倒车请注意",
    "04_turn_left.wav": "左转",
    "05_turn_right.wav": "右转",
    "06_avoiding_obstacle.wav": "机器人避障中",
}
AUDIO_SUFFIXES = {".wav", ".mp3", ".ogg"}


def default_target_dir():
    return Path.home() / ".local" / "share" / "robot_voice_announcer" / "audio"


def find_reference_dir(cli_value):
    if cli_value:
        path = Path(cli_value).expanduser().resolve()
        return path if path.is_dir() else None

    source_tree = Path(__file__).resolve().parent.parent / "audio"
    if source_tree.is_dir():
        return source_tree

    try:
        import rospkg

        path = Path(rospkg.RosPack().get_path("robot_voice_announcer")) / "audio"
        return path.resolve() if path.is_dir() else None
    except Exception:
        return None


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inspect_wav(path):
    try:
        with wave.open(str(path), "rb") as wav_file:
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            sample_rate = wav_file.getframerate()
            frames = wav_file.getnframes()
            compression = wav_file.getcomptype()
    except (OSError, wave.Error) as exc:
        return False, "无法读取 WAV: {}".format(exc)

    duration = frames / float(sample_rate) if sample_rate else 0.0
    details = "{}声道, {}位, {} Hz, {:.2f}秒".format(
        channels, sample_width * 8, sample_rate, duration
    )
    valid = (
        channels in (1, 2)
        and sample_width == 2
        and sample_rate >= 8000
        and frames > 0
        and compression == "NONE"
    )
    if not valid:
        return False, details + "，格式不符合要求"
    return True, details


def sound_device_status(player):
    executable = shutil.which(player)
    if not executable:
        return False, "找不到 {}，请安装 alsa-utils".format(player)

    try:
        result = subprocess.run(
            [executable, "-l"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, "声卡查询失败: {}".format(exc)

    if result.returncode != 0:
        output = (result.stdout or "").strip().replace("\n", " ")
        return False, "未检测到可用声卡: {}".format(output or "aplay -l 失败")
    return True, "播放器和 ALSA 声卡可用"


def parse_args():
    parser = argparse.ArgumentParser(
        description="只读检查机器人语音目录、六个 WAV 文件及 ALSA 播放环境。"
    )
    parser.add_argument(
        "--target-dir",
        default=os.environ.get("ROBOT_VOICE_DIR", str(default_target_dir())),
        help="要检查的机器人语音目录（默认: %(default)s）",
    )
    parser.add_argument(
        "--reference-dir",
        help="原始六个 WAV 所在目录；省略时自动从 ROS 包查找",
    )
    parser.add_argument(
        "--skip-audio-device",
        action="store_true",
        help="跳过 aplay 和 ALSA 声卡检查",
    )
    parser.add_argument(
        "--player-command",
        default="aplay",
        help="播放命令（默认: %(default)s）",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    target_dir = Path(args.target_dir).expanduser().resolve()
    reference_dir = find_reference_dir(args.reference_dir)
    errors = []
    warnings = []

    print("机器人语音目录检查")
    print("目录: {}".format(target_dir))

    if not target_dir.exists():
        print("[失败] 目录不存在")
        print("请先运行: rosrun robot_voice_announcer install_voice_files.py")
        return 1
    if not target_dir.is_dir():
        print("[失败] 目标路径不是目录")
        return 1

    readable = os.access(str(target_dir), os.R_OK | os.X_OK)
    writable = os.access(str(target_dir), os.W_OK)
    print("[{}] 目录可读".format("通过" if readable else "失败"))
    print("[{}] 目录可写".format("通过" if writable else "警告"))
    if not readable:
        errors.append("语音目录不可读")
    if not writable:
        warnings.append("语音目录不可写，后续重新导入可能失败")

    print("\n语音文件:")
    for filename, phrase in VOICE_FILES.items():
        path = target_dir / filename
        if not path.is_file():
            print("[失败] {} - 缺失（{}）".format(filename, phrase))
            errors.append("缺少 {}".format(filename))
            continue
        if not os.access(str(path), os.R_OK):
            print("[失败] {} - 不可读".format(filename))
            errors.append("{} 不可读".format(filename))
            continue

        valid, details = inspect_wav(path)
        print("[{}] {} - {} - {}".format(
            "通过" if valid else "失败", filename, phrase, details
        ))
        if not valid:
            errors.append("{} 格式错误".format(filename))
            continue

        if reference_dir and (reference_dir / filename).is_file():
            if sha256(path) != sha256(reference_dir / filename):
                warnings.append("{} 与包内原始文件内容不同".format(filename))

    extras = sorted(
        item.name for item in target_dir.iterdir()
        if item.is_file()
        and item.suffix.lower() in AUDIO_SUFFIXES
        and item.name not in VOICE_FILES
    )
    if extras:
        warnings.append("目录中还有其他语音文件: {}".format(", ".join(extras)))

    if reference_dir:
        print("\n参考目录: {}".format(reference_dir))
    else:
        warnings.append("未找到包内参考语音，只完成了格式检查")

    if not args.skip_audio_device:
        sound_ok, sound_message = sound_device_status(args.player_command)
        print("[{}] {}".format("通过" if sound_ok else "失败", sound_message))
        if not sound_ok:
            errors.append(sound_message)

    if warnings:
        print("\n警告:")
        for item in warnings:
            print("- {}".format(item))

    if errors:
        print("\n检查结果: 失败（{} 项）".format(len(errors)))
        for item in errors:
            print("- {}".format(item))
        return 1

    print("\n检查结果: 通过，6 个语音文件均可使用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
