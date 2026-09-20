#!/usr/bin/env python3
"""Pure helpers for keeping the robot catalog scoped to the active map."""


MAP_KEYS = {
    "map", "map_name", "mapname", "map_id", "mapid", "map_uuid", "mapuuid",
}
MAP_IDENTITY_KEYS = MAP_KEYS | {"name", "id", "uuid"}


def _normalized(value):
    if isinstance(value, bool) or value is None:
        return ""
    if isinstance(value, (str, int, float)):
        return "".join(str(value).split()).casefold()
    return ""


def _identity_values(value, keys):
    """Collect scalar identities below selected keys from nested API payloads."""
    found = set()

    def add_scalars(node):
        if isinstance(node, dict):
            for child_key, child in node.items():
                if str(child_key).lower() in MAP_IDENTITY_KEYS:
                    normalized = _normalized(child)
                    if normalized:
                        found.add(normalized)
                if isinstance(child, (dict, list)):
                    add_scalars(child)
        elif isinstance(node, list):
            for child in node:
                add_scalars(child)
        else:
            normalized = _normalized(node)
            if normalized:
                found.add(normalized)

    def walk(node):
        if isinstance(node, dict):
            for key, child in node.items():
                if str(key).lower() in keys:
                    add_scalars(child)
                if isinstance(child, (dict, list)):
                    walk(child)
        elif isinstance(node, list):
            for child in node:
                walk(child)

    walk(value)
    return found


def map_identities(current_map):
    """Return the names/IDs that can identify the current map."""
    if isinstance(current_map, dict):
        found = set()
        for key, value in current_map.items():
            if str(key).lower() in MAP_IDENTITY_KEYS:
                normalized = _normalized(value)
                if normalized:
                    found.add(normalized)
                if isinstance(value, (dict, list)):
                    found.update(_identity_values({key: value}, {str(key).lower()}))
        return found
    normalized = _normalized(current_map)
    return {normalized} if normalized else set()


def task_name(task):
    if not isinstance(task, dict):
        return ""
    for key in ("name", "task_name", "taskName", "main_task_name", "mainTaskName"):
        value = task.get(key)
        if value not in (None, ""):
            return str(value).strip()
    return ""


def current_map_tasks(tasks, current_map):
    """Return every real task belonging to the current map, preserving API order.

    Some chassis versions return map metadata on the task, some put it on a
    nested movement action, and some already scope get_task_list to the current
    map and return no map field at all. The last form is accepted only when the
    complete response is unscoped; mixed scoped/unscoped responses remain
    fail-closed to avoid starting a task from another map.
    """
    records = [item for item in tasks if isinstance(item, dict)] if isinstance(tasks, list) else []
    current = map_identities(current_map)
    if not records or not current:
        return []

    references = [_identity_values(record, MAP_KEYS) for record in records]
    response_is_unscoped = not any(references)
    selected = []
    seen = set()
    for record, task_maps in zip(records, references):
        name = task_name(record)
        matches_explicit_map = bool(task_maps & current)
        if matches_explicit_map or (not task_maps and response_is_unscoped):
            identity = name or repr(record)
            if identity not in seen:
                selected.append(record)
                seen.add(identity)
    return selected
