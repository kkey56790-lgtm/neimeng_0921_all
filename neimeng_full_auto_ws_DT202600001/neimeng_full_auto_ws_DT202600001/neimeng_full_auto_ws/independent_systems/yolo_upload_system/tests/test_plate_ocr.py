"""OCR流程测试：模拟模型与OpenCV，不需要摄像头或下载权重。"""
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from plate_ocr import PlateOCR
from test_snapshot_upload import SnapshotTests


class PlateOCRTests(unittest.TestCase):
    def setUp(self):
        self.reader = Mock()
        self.reader.recognize.return_value = [(None, 'b 1001-', .95)]
        self.cv = Mock()
        with patch.dict(sys.modules, cv2=self.cv):
            self.ocr = PlateOCR(reader=self.reader)
        self.frame = Mock()
        self.frame.shape = (480, 640, 3)
        self.frame.__getitem__ = Mock(return_value=Mock())

    def test_correct_class_and_threshold(self):
        objects = [dict(class_name='plate', class_id=3, confidence=.6, bbox=[-2, 10, 100, 40]),
                   dict(class_name='extinguisher', class_id=2, confidence=.9, bbox=[0,0,50,50]),
                   dict(class_name='plate', confidence=.59, bbox=[0,0,50,50])]
        self.assertTrue(self.ocr.recognize_frame(self.frame, objects))
        self.assertEqual(objects[0]['content'], 'B1001')
        self.assertEqual(objects[0]['result_name'], '车牌号')
        self.assertNotIn('content', objects[1])
        self.assertEqual(objects[2]['content'], '')
        self.reader.recognize.assert_called_once()

    def test_no_cached_plate_and_low_ocr_confidence(self):
        obj = dict(class_name='plate', confidence=.9, bbox=[0,0,50,50])
        self.ocr.recognize_frame(self.frame, [obj])
        self.reader.recognize.return_value = [(None, 'wrong', .2)]
        self.ocr.recognize_frame(self.frame, [obj])
        self.assertEqual(obj['content'], '')
        self.reader.recognize.side_effect = RuntimeError('model failure')
        self.ocr.recognize_frame(self.frame, [obj])
        self.assertEqual(obj['content'], '')

    def test_bad_boxes_skip_ocr(self):
        for bbox in ([0,0,0,0], [float('nan'),0,20,20], [100,100,20,20]):
            self.assertFalse(self.ocr.recognize_frame(self.frame, [dict(class_name='plate', confidence=.9, bbox=bbox)]))
        self.reader.recognize.assert_not_called()

    def test_disabled_needs_no_easyocr(self):
        with patch.dict(os.environ, PLATE_OCR_ENABLED='0'):
            self.assertIsNone(PlateOCR.from_env())


class UploadOCRTests(SnapshotTests):
    def test_ocr_text_goes_into_saved_image_metadata(self):
        self.u.items = [dict(id=3, name='plate', display_name='车牌')]
        self.status()
        def recognize(image, objects):
            objects[0].update(content='B1001', result_name='车牌号')
        self.u.plate_ocr = Mock()
        self.u.plate_ocr.recognize_frame.side_effect = recognize
        self.collect('plate')
        self.assertEqual(self.records()[0]['imageMetadata']['result'],
                         [dict(name='车牌号', content='B1001')])
        self.u.plate_ocr.recognize_frame.assert_called_once()
