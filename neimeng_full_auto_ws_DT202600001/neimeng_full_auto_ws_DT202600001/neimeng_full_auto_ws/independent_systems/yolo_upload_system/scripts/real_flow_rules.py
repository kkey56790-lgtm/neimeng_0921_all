#!/usr/bin/env python3
"""Pure helpers for numbered real-inspection task and telemetry matching."""

import re


TASK_NUMBER_RE = re.compile(r"任务\s*0*(\d+)(?!\d)", re.IGNORECASE)
START_POINT_RE = re.compile(r"起始点\s*0*(\d+)(?!\d)", re.IGNORECASE)


def compact(value):
    return re.sub(r"\s+", "", str(value or "")).lower()


def task_number(value):
    match = TASK_NUMBER_RE.search(str(value or ""))
    return int(match.group(1)) if match else None


def start_point_number(value):
    match = START_POINT_RE.search(str(value or ""))
    return int(match.group(1)) if match else None


def task_area_prefix(value):
    text = str(value or "").strip()
    match = TASK_NUMBER_RE.search(text)
    return text[:match.start()].strip(" -_/") if match else ""


def available_task_numbers(names, maximum=10):
    """Return the real positive task numbers present in the current map catalog."""
    maximum = max(1, int(maximum))
    return sorted({
        number for number in (task_number(name) for name in names)
        if number is not None and 1 <= number <= maximum
    })


def next_task_number(current, names, maximum=10):
    """Advance through existing map tasks and wrap the final one to task 1."""
    current = int(current or 0)
    numbers = available_task_numbers(names, maximum)
    if numbers:
        for number in numbers:
            if number > current:
                return number
        return 1 if 1 in numbers else numbers[0]
    maximum = max(1, int(maximum))
    return 1 if current >= maximum else max(1, current + 1)


class TaskCatalogResolver:
    def __init__(self, max_task_number=10):
        self.max_task_number = int(max_task_number)
        self.names = []

    def update(self, names):
        self.names = list(dict.fromkeys(
            str(name).strip() for name in names if str(name).strip()
        ))

    def resolve_number(self, number, preferred_area="", requested_name=""):
        number = int(number)
        if number < 0 or number > self.max_task_number:
            raise ValueError("任务编号超出范围：{}".format(number))
        requested = str(requested_name or "").strip()
        if requested and requested in self.names and task_number(requested) == number:
            return requested
        candidates = [name for name in self.names if task_number(name) == number]
        if not candidates:
            raise ValueError("真实任务目录中找不到关键词“任务{}”".format(number))
        requested_area = task_area_prefix(requested)
        for area in (requested_area, preferred_area):
            normalized_area = compact(area)
            if not normalized_area:
                continue
            matching = [name for name in candidates if normalized_area in compact(task_area_prefix(name))]
            if len(matching) == 1:
                return matching[0]
            if matching:
                candidates = matching
                break
        if len(candidates) > 1:
            raise ValueError(
                "任务{}匹配到多个区域任务，请下发完整名称：{}".format(
                    number, "、".join(sorted(candidates))))
        return candidates[0]

    def resolve_keywords(self, keywords, preferred_area=""):
        normalized_keywords = [compact(value) for value in keywords if compact(value)]
        candidates = [
            name for name in self.names
            if any(keyword in compact(name) for keyword in normalized_keywords)
        ]
        area = compact(preferred_area)
        if area:
            matching = [name for name in candidates if area in compact(name)]
            if matching:
                candidates = matching
        if not candidates:
            raise ValueError("真实任务目录中找不到任务关键词：{}".format("/".join(keywords)))
        if len(candidates) > 1:
            raise ValueError("回充任务匹配不唯一：{}".format("、".join(sorted(candidates))))
        return candidates[0]


def _walk_scalars(value, path=""):
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = path + "/" + str(key).lower()
            if isinstance(child, (dict, list)):
                for item in _walk_scalars(child, child_path):
                    yield item
            else:
                yield child_path, child
    elif isinstance(value, list):
        for index, child in enumerate(value):
            for item in _walk_scalars(child, path + "/" + str(index)):
                yield item


def extract_battery_percent(telemetry):
    scored = []
    for path, value in _walk_scalars(telemetry):
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        leaf = path.rsplit("/", 1)[-1]
        score = 0
        if leaf in ("soc", "battery_soc", "batterypercent", "battery_percent"):
            score = 100
        elif leaf in ("battery", "power", "electricity", "capacity_percent"):
            score = 70
        elif "battery" in path and leaf in ("value", "percent", "percentage", "capacity"):
            score = 80
        if score and 0.0 <= number <= 100.0:
            scored.append((score, number))
    return max(scored, key=lambda item: item[0])[1] if scored else None


def _truthy_obstacle(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    normalized = compact(value)
    if normalized in ("true", "yes", "1", "blocked", "obstacle", "detected", "有障碍", "避障"):
        return True
    if normalized in ("false", "no", "0", "clear", "none", "无障碍", "正常"):
        return False
    return None


def extract_obstacle_state(telemetry):
    candidates = []
    for path, value in _walk_scalars(telemetry):
        leaf = path.rsplit("/", 1)[-1]
        if leaf not in ("obstacle", "obstacle_state", "avoidance", "blocked"):
            continue
        parsed = _truthy_obstacle(value)
        if parsed is not None:
            candidates.append(parsed)
    return candidates[0] if candidates else None

