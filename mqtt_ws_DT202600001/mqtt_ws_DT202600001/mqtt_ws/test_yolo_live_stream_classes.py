"""无需摄像头、MQTT 服务或 RKNN 硬件的类别映射回归检查。"""
import ast
import copy
import pathlib
import threading
import unittest


SOURCE = pathlib.Path(__file__).with_name("yolo_live_stream_mqtt.py")
tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
selected = []
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "DETECTION_CLASSES" for t in node.targets):
        selected.append(node)
    elif isinstance(node, ast.FunctionDef) and node.name in (
            "normalize_detection", "handle_mqtt_command", "draw_detection"):
        selected.append(node)
scope = {}
exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"), scope)


class DetectionClassesTest(unittest.TestCase):
    def test_new_model_replaces_stale_upstream_labels(self):
        original = {"timestamp": 123, "objects": [
            {"class_id": i, "class_name": "old", "class_name_cn": "旧标签",
             "confidence": 0.9, "bbox": [1, 2, 30, 40]} for i in range(11)]}
        saved = copy.deepcopy(original)
        result = scope["normalize_detection"](original)
        self.assertEqual(original, saved)
        self.assertEqual(result["object_count"], 11)
        self.assertEqual(result["timestamp"], 123)
        self.assertEqual([o["class_name"] for o in result["objects"]], [
            "chocks", "extinguisher", "plate", "tag", "light", "support",
            "screw", "tank", "lamp_broken", "box_broken", "warning"])
        self.assertEqual(result["objects"][10]["bbox"], [1, 2, 30, 40])

    def test_invalid_ids_do_not_use_negative_index_or_truncate(self):
        result = scope["normalize_detection"]({"objects": [
            {"class_id": i} for i in (-1, 11, 1.5, True, None)]})
        self.assertTrue(all(o["class_name"].startswith("unknown_") for o in result["objects"]))

    def test_string_id_and_malformed_objects(self):
        result = scope["normalize_detection"]({"objects": [None, {"class_id": "10"}]})
        self.assertEqual(result["object_count"], 1)
        self.assertEqual(result["objects"][0]["class_name"], "warning")
        self.assertEqual(scope["normalize_detection"]({"objects": None})["objects"], [])

    def test_mqtt_reply_contains_new_labels(self):
        published = []
        scope.update(detection_lock=threading.Lock(), ROBOT_CODE="test",
                     MQTT_REPLY_TOPIC="reply", mqtt_publish_json=lambda *args: published.append(args),
                     latest_detection=scope["normalize_detection"]({"objects": [{"class_id": 8}]}))
        scope["handle_mqtt_command"](None, {"method": "get_detection"})
        obj = published[0][1]["data"]["objects"][0]
        self.assertEqual(obj["class_name"], "lamp_broken")
        self.assertEqual(obj["class_name_cn"], "车灯破损")


if __name__ == "__main__":
    unittest.main()
