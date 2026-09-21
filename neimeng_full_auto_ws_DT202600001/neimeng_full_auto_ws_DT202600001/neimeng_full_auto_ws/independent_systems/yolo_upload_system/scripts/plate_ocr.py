"""对现有YOLO视频帧的车牌框做EasyOCR，不加载第二个检测器。"""
import math
import os
import re


class PlateOCR:
    def __init__(self, reader=None, min_confidence=0.5, model_dir=None,
                 download=True, class_names=('plate', 'license_plate', '车牌', '车牌号')):
        import cv2
        self.cv2 = cv2
        self.min_confidence = float(min_confidence)
        if not 0 <= self.min_confidence <= 1:
            raise ValueError('PLATE_OCR_MIN_CONFIDENCE必须在0到1之间')
        self.class_names = {name.strip().lower() for name in class_names}
        if reader is None:
            import easyocr
            reader = easyocr.Reader(['en'], gpu=False, detector=False, verbose=False,
                                    model_storage_directory=model_dir,
                                    download_enabled=download)
        self.reader = reader

    @classmethod
    def from_env(cls):
        if os.environ.get('PLATE_OCR_ENABLED', '0').lower() not in ('1', 'true', 'yes'):
            return None
        return cls(min_confidence=float(os.environ.get('PLATE_OCR_MIN_CONFIDENCE', '0.5')),
                   model_dir=os.environ.get('PLATE_OCR_MODEL_DIR') or None,
                   download=os.environ.get('PLATE_OCR_DOWNLOAD', '1').lower() in ('1', 'true', 'yes'),
                   class_names=os.environ.get('PLATE_OCR_CLASSES', 'plate,license_plate,车牌,车牌号').split(','))

    def recognize_frame(self, frame, objects):
        """返回是否包含合格车牌框；正文只属于当前帧当前框，不缓存串车。"""
        cv2 = self.cv2
        found = False
        height, width = frame.shape[:2]
        for obj in objects:
            if not any(str(obj.get(key, '')).strip().lower() in self.class_names
                       for key in ('canonical_name', 'class_name', 'class_name_cn')):
                continue
            obj['content'] = ''
            obj['result_name'] = '车牌号'
            try:
                confidence = float(obj.get('confidence', 0))
                if not math.isfinite(confidence) or confidence < 0.6:
                    continue
                coords = [float(v) for v in obj['bbox']]
                if len(coords) != 4 or not all(math.isfinite(v) for v in coords):
                    continue
                x1, y1, x2, y2 = [int(v) for v in coords]
                x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
                y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
                if x2 - x1 < 2 or y2 - y1 < 2:
                    continue
            except (KeyError, ValueError, TypeError, OverflowError):
                continue
            found = True
            crop = frame[y1:y2, x1:x2]
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (min(1024, max(2, int((x2-x1)*64/(y2-y1)))), 64),
                              interpolation=cv2.INTER_CUBIC)
            try:
                results = self.reader.recognize(gray, decoder='greedy', detail=1,
                    allowlist='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789')
                text = ''.join(str(row[1]) for row in results
                               if math.isfinite(float(row[2])) and float(row[2]) >= self.min_confidence)
                obj['content'] = re.sub(r'[^A-Za-z0-9]', '', text).upper()
            except Exception as exc:
                obj['ocr_error'] = type(exc).__name__
                print('车牌OCR失败，本帧不上报车牌文字:', type(exc).__name__, flush=True)
        return found
