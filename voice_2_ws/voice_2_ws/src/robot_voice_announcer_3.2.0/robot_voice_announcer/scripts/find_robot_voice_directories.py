#!/usr/bin/env python3
"""Read-only discovery of existing robot audio directories on Ubuntu."""

import argparse
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

try:
    import pwd
except ImportError:  # Allows validation on non-POSIX development machines.
    pwd = None


AUDIO_SUFFIXES = {".wav", ".mp3", ".ogg", ".flac", ".m4a", ".aac"}
DEFAULT_ROOTS = ("/home", "/opt", "/usr/local", "/data", "/mnt", "/media")
SKIP_DIR_NAMES = {
    ".cache",
    ".git",
    "__pycache__",
    "node_modules",
    "build",
    "devel",
    "log",
    "logs",
}
PATH_KEYWORDS = {
    "voice": 35,
    "voices": 35,
    "audio": 30,
    "sound": 25,
    "sounds": 25,
    "speech": 35,
    "tts": 40,
    "prompt": 30,
    "prompts": 30,
    "announce": 30,
    "alarm": 15,
    "语音": 45,
    "播报": 45,
    "提示音": 40,
}


def human_size(size):
    value = float(size)
    for unit in ("B", "KB", "MB", "GB"):
        if value < 1024.0 or unit == "GB":
            return "{:.1f} {}".format(value, unit)
        value /= 1024.0
    return "{} B".format(size)


def existing_default_roots():
    roots = [Path(root) for root in DEFAULT_ROOTS if Path(root).is_dir()]
    try:
        current_home = Path(pwd.getpwuid(os.getuid()).pw_dir) if pwd else Path.home()
        if current_home.is_dir() and current_home not in roots:
            roots.insert(0, current_home)
    except (KeyError, OSError):
        pass
    return roots


def depth_from_root(path, root):
    try:
        return len(Path(path).relative_to(root).parts)
    except ValueError:
        return 999999


def scan_audio(roots, max_depth):
    found = defaultdict(list)
    denied = 0

    def onerror(_error):
        nonlocal denied
        denied += 1

    for root in roots:
        if not root.is_dir():
            continue
        for current, dirs, files in os.walk(str(root), topdown=True, followlinks=False,
                                            onerror=onerror):
            current_path = Path(current)
            depth = depth_from_root(current_path, root)
            if depth >= max_depth:
                dirs[:] = []
            else:
                dirs[:] = [
                    name for name in dirs
                    if name not in SKIP_DIR_NAMES
                    and not Path(current_path, name).is_symlink()
                ]

            for filename in files:
                if Path(filename).suffix.lower() in AUDIO_SUFFIXES:
                    path = current_path / filename
                    try:
                        size = path.stat().st_size
                    except OSError:
                        size = -1
                    found[current_path].append((filename, size))
    return found, denied


def process_audio_references():
    """Return audio paths currently opened by running processes."""
    references = defaultdict(set)
    proc = Path("/proc")
    if not proc.is_dir():
        return references

    for process_dir in proc.iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            process_name = (process_dir / "comm").read_text(
                encoding="utf-8", errors="replace"
            ).strip()
            for fd in (process_dir / "fd").iterdir():
                try:
                    target = fd.resolve(strict=True)
                except (OSError, RuntimeError):
                    continue
                if target.suffix.lower() in AUDIO_SUFFIXES:
                    references[target].add("{}[{}]".format(process_name, process_dir.name))
        except (OSError, PermissionError):
            continue
    return references


def score_directory(directory, files, open_references):
    path_text = str(directory).lower()
    score = min(30, len(files) * 2)
    reasons = []

    for keyword, points in PATH_KEYWORDS.items():
        if keyword in path_text:
            score += points
            reasons.append("路径含 {}".format(keyword))

    live_processes = set()
    for filename, _size in files:
        live_processes.update(open_references.get(directory / filename, set()))
    if live_processes:
        score += 100
        reasons.append("正被进程使用: {}".format(", ".join(sorted(live_processes))))

    names = " ".join(filename.lower() for filename, _size in files)
    if re.search(r"(stop|move|turn|left|right|back|obstacle|巡检|停车|倒车|左转|右转|避障)", names):
        score += 35
        reasons.append("文件名类似机器人提示语音")

    if len(files) >= 3:
        reasons.append("包含 {} 个音频".format(len(files)))
    return score, reasons


def parse_args():
    parser = argparse.ArgumentParser(
        description="只读扫描 Ubuntu，找出机器人原有语音文件的可能目录。"
    )
    parser.add_argument(
        "--root",
        action="append",
        help="扫描根目录，可重复指定；省略时扫描常用机器人程序目录",
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        default=10,
        help="每个根目录最大扫描深度（默认: %(default)s）",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="最多显示多少个候选目录（默认: %(default)s）",
    )
    parser.add_argument(
        "--show-files",
        type=int,
        default=8,
        help="每个目录最多显示多少个文件名（默认: %(default)s）",
    )
    parser.add_argument(
        "--no-process-check",
        action="store_true",
        help="不检查运行进程当前打开的音频文件",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.max_depth < 1 or args.top < 1 or args.show_files < 1:
        print("参数必须为正整数", file=sys.stderr)
        return 2

    roots = ([Path(item).expanduser().resolve() for item in args.root]
             if args.root else existing_default_roots())
    roots = list(dict.fromkeys(root for root in roots if root.is_dir()))
    if not roots:
        print("没有可扫描的目录", file=sys.stderr)
        return 2

    print("机器人语音目录自动搜索")
    print("扫描范围:")
    for root in roots:
        print("- {}".format(root))
    print("最大深度: {}".format(args.max_depth))
    print("正在扫描，只进行读取，不会修改文件……")

    found, denied = scan_audio(roots, args.max_depth)
    open_references = (defaultdict(set) if args.no_process_check
                       else process_audio_references())

    ranked = []
    for directory, files in found.items():
        score, reasons = score_directory(directory, files, open_references)
        ranked.append((score, str(directory), directory, files, reasons))
    ranked.sort(key=lambda item: (-item[0], item[1]))

    if not ranked:
        print("\n未在默认范围内找到音频文件。")
        print("可执行全盘搜索: sudo python3 {} --root / --max-depth 20".format(
            Path(__file__).name
        ))
        return 1

    print("\n候选语音目录（分数越高越可能是机器人语音目录）:")
    for index, (score, _sort_path, directory, files, reasons) in enumerate(
            ranked[:args.top], start=1):
        total_size = sum(max(size, 0) for _name, size in files)
        print("\n{}. {}".format(index, directory))
        print("   评分: {}  文件数: {}  总大小: {}".format(
            score, len(files), human_size(total_size)
        ))
        if reasons:
            print("   依据: {}".format("；".join(reasons)))
        samples = sorted(name for name, _size in files)[:args.show_files]
        print("   示例: {}".format(", ".join(samples)))

    print("\n最可能目录: {}".format(ranked[0][2]))
    if denied:
        print("注意: 有 {} 个目录因权限不足未能读取；可使用 sudo 重新检查。".format(denied))
    print("确认候选文件内容时可执行: aplay '目录/文件.wav'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
