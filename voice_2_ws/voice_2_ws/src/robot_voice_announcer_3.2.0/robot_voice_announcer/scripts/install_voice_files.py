#!/usr/bin/env python3
"""Back up, replace and verify robot WAV files through the HTTP V2.4 API."""

import argparse
import datetime
import sys
import wave
from pathlib import Path

import requests


VOICE_FILES = (
    "01_inspecting.wav",
    "02_stopped.wav",
    "03_reversing.wav",
    "04_turn_left.wav",
    "05_turn_right.wav",
    "06_avoiding_obstacle.wav",
)

# The robot firmware rejects deletion of these built-in safety prompts.
PROTECTED_REMOTE_FILES = {
    "turn_left.wav",
    "turn_right.wav",
    "stop_car.wav",
}


def find_source_dir(cli_value):
    if cli_value:
        return Path(cli_value).expanduser().resolve()
    source_tree = Path(__file__).resolve().parent.parent / "audio"
    if source_tree.is_dir():
        return source_tree
    try:
        import rospkg
        return (Path(rospkg.RosPack().get_path("robot_voice_announcer")) / "audio").resolve()
    except Exception as exc:
        raise RuntimeError("找不到随包提供的 audio 目录，请使用 --source-dir 指定") from exc


def validate_source(source_dir):
    missing = [name for name in VOICE_FILES if not (source_dir / name).is_file()]
    if missing:
        raise RuntimeError("源语音目录缺少文件: " + ", ".join(missing))
    for name in VOICE_FILES:
        path = source_dir / name
        try:
            with wave.open(str(path), "rb") as wav_file:
                valid = (wav_file.getnchannels() in (1, 2)
                         and wav_file.getsampwidth() == 2
                         and wav_file.getframerate() >= 8000
                         and wav_file.getnframes() > 0
                         and wav_file.getcomptype() == "NONE")
            if not valid:
                raise RuntimeError("必须是有效的 16 位 PCM WAV")
        except (OSError, wave.Error, RuntimeError) as exc:
            raise RuntimeError("无效语音文件 {}: {}".format(path, exc)) from exc


class RobotVoiceApi:
    def __init__(self, robot_ip, port, timeout):
        self.base_url = "http://{}:{}".format(robot_ip, port)
        self.timeout = timeout
        self.session = requests.Session()

    @staticmethod
    def require_success(response, operation):
        response.raise_for_status()
        try:
            payload = response.json()
        except ValueError as exc:
            raise RuntimeError("{} 返回的不是 JSON".format(operation)) from exc
        if not isinstance(payload, dict) or not payload.get("result", False):
            detail = payload.get("data", payload) if isinstance(payload, dict) else payload
            raise RuntimeError("{} 失败: {}".format(operation, detail))
        return payload

    def list_files(self):
        response = self.session.get(self.base_url + "/system_manager/get_wav_list",
                                    timeout=self.timeout)
        data = self.require_success(response, "获取语音列表").get("data", [])
        if not isinstance(data, list):
            raise RuntimeError("获取语音列表失败: data 不是数组")
        return [item for item in data if isinstance(item, dict) and item.get("name")]

    def download(self, filename):
        response = self.session.post(
            self.base_url + "/system_manager/download_wav",
            params={"file_name": filename}, timeout=self.timeout)
        response.raise_for_status()
        if "json" in response.headers.get("Content-Type", "").lower():
            self.require_success(response, "下载 {}".format(filename))
            raise RuntimeError("下载 {} 未返回 WAV 数据".format(filename))
        return response.content

    def delete(self, filename):
        response = self.session.post(
            self.base_url + "/system_manager/delete_wav_data",
            json={"file_name": filename}, timeout=self.timeout)
        self.require_success(response, "删除 {}".format(filename))

    def upload(self, filename, content):
        response = self.session.post(
            self.base_url + "/system_manager/upload_wav_data",
            params={"file_name": filename}, data=content,
            headers={"Content-Type": "application/octet-stream"}, timeout=self.timeout)
        self.require_success(response, "上传 {}".format(filename))


def safe_backup_name(filename):
    safe = filename.replace("/", "_").replace("\\", "_").strip()
    if safe in ("", ".", ".."):
        raise RuntimeError("服务器返回了不安全的文件名: {!r}".format(filename))
    return safe


def default_backup_dir():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / ("robot_voice_backup_" + timestamp)


def parse_args():
    parser = argparse.ArgumentParser(
        description="通过机器人 HTTP V2.4 接口备份、删除并上传六个语音文件。")
    parser.add_argument("--robot-ip", default="192.168.2.100", help="机器人 HTTP IP")
    parser.add_argument("--port", type=int, default=7999, help="机器人 HTTP 端口")
    parser.add_argument("--timeout", type=float, default=8.0, help="单次 HTTP 超时秒数")
    parser.add_argument("--source-dir", help="本地六个 WAV 所在目录")
    parser.add_argument("--backup-dir", help="旧语音下载备份目录")
    parser.add_argument("--no-backup", action="store_true", help="跳过旧语音备份；不推荐")
    parser.add_argument("--dry-run", action="store_true",
                        help="只列出将删除和上传的文件，不执行变更")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        source_dir = find_source_dir(args.source_dir)
        validate_source(source_dir)
        api = RobotVoiceApi(args.robot_ip, args.port, args.timeout)
        remote_files = api.list_files()
    except Exception as exc:
        print("准备失败: {}".format(exc), file=sys.stderr)
        return 1

    remote_names = [str(item["name"]) for item in remote_files]
    print("机器人: http://{}:{}".format(args.robot_ip, args.port))
    print("服务器现有语音 {} 个:".format(len(remote_names)))
    for name in remote_names:
        print("  {}".format(name))
    print("将上传:")
    for name in VOICE_FILES:
        print("  {}".format(name))
    if args.dry_run:
        print("dry-run 完成，未删除或上传文件。")
        return 0

    backup_dir = None
    try:
        if remote_names and not args.no_backup:
            backup_dir = (Path(args.backup_dir).expanduser().resolve()
                          if args.backup_dir else default_backup_dir().resolve())
            backup_dir.mkdir(parents=True, exist_ok=False)
            print("正在备份旧语音到 {}".format(backup_dir))
            for name in remote_names:
                content = api.download(name)
                destination = backup_dir / safe_backup_name(name)
                destination.write_bytes(content)
                print("  已备份 {} ({} B)".format(name, len(content)))

        print("正在删除服务器现有语音……")
        for name in remote_names:
            if name in PROTECTED_REMOTE_FILES:
                print("  固件保护，保留 {}".format(name))
                continue
            api.delete(name)
            print("  已删除 {}".format(name))

        print("正在上传六个新语音……")
        for name in VOICE_FILES:
            content = (source_dir / name).read_bytes()
            api.upload(name, content)
            print("  已上传 {} ({} B)".format(name, len(content)))

        final_names = {str(item.get("name")) for item in api.list_files()}
        missing = [name for name in VOICE_FILES if name not in final_names]
        if missing:
            raise RuntimeError("上传后列表仍缺少: {}".format(", ".join(missing)))
    except Exception as exc:
        print("语音更新失败: {}".format(exc), file=sys.stderr)
        if backup_dir:
            print("旧语音备份保留在: {}".format(backup_dir), file=sys.stderr)
        return 1

    print("语音更新成功，服务器已存在全部六个文件。")
    if backup_dir:
        print("旧语音备份: {}".format(backup_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
