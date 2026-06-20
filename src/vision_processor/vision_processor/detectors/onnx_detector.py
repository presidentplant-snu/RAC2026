import numpy as np
import cv2 as cv

import onnxruntime


class OnnxDetector():
    """YOLO ONNX detector.

    Assumes an end-to-end model with NMS baked in that emits three outputs
    (boxes, conf, cls), as produced for the `*_3out` models in ~/onnx/models:
        boxes : (1, N, 4)  -> x1, y1, x2, y2 in letterboxed pixel coords
        conf  : (1, N, 1)  -> confidence
        cls   : (1, N, 1)  -> class id

    `detect()` returns the pixel center [cx, cy] of the highest-confidence
    detection (optionally filtered to `target_class`) in the original image's
    coordinates, or None. This mirrors ArucoDetector so it is a drop-in for the
    vision_processor tracking loop.
    """

    def __init__(
        self,
        model_path: str,
        img_size: int = 1280,
        conf_thres: float = 0.25,
        target_class: int | None = None,
        providers: list | None = None,
        provider_options: list | None = None,
    ) -> None:
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.target_class = target_class

        # Default to the QNN HTP backend (as in the reference), falling back to
        # CPU if QNN isn't available in this runtime.
        if providers is None:
            available = onnxruntime.get_available_providers()
            if "QNNExecutionProvider" in available:
                providers = ["QNNExecutionProvider"]
                provider_options = [{"backend_path": "libQnnHtp.so"}]
            else:
                providers = ["CPUExecutionProvider"]
                provider_options = None

        options = onnxruntime.SessionOptions()
        options.add_session_config_entry("session.disable_cpu_ep_fallback", "0")

        self.session = onnxruntime.InferenceSession(
            model_path,
            sess_options=options,
            providers=providers,
            provider_options=provider_options,
        )
        self.input_name = self.session.get_inputs()[0].name

        # Letterbox geometry, cached per input frame size.
        self._geom_for: tuple[int, int] | None = None
        self._scale = 1.0
        self._top = 0
        self._left = 0

    def _update_letterbox(self, h: int, w: int) -> None:
        """Recompute letterbox scale/padding when the frame size changes."""
        if self._geom_for == (h, w):
            return
        scale = min(self.img_size / w, self.img_size / h)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        self._scale = scale
        self._top = int(round((self.img_size - new_h) / 2 - 0.1))
        self._left = int(round((self.img_size - new_w) / 2 - 0.1))
        self._new_w, self._new_h = new_w, new_h
        self._geom_for = (h, w)

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        resized = cv.resize(image, (self._new_w, self._new_h),
                            interpolation=cv.INTER_LINEAR)
        canvas = np.full((self.img_size, self.img_size, 3), 114, dtype=np.uint8)
        canvas[self._top:self._top + self._new_h,
               self._left:self._left + self._new_w] = resized

        blob = canvas[:, :, ::-1].astype(np.float32) / 255.0   # BGR -> RGB
        blob = np.transpose(blob, (2, 0, 1))[None]             # 1x3xHxW
        return np.ascontiguousarray(blob)

    def detect(self, image: np.ndarray | None) -> np.ndarray | None:
        if image is None:
            return None

        h, w = image.shape[:2]
        self._update_letterbox(h, w)

        blob = self._preprocess(image)
        boxes, conf, cls = self.session.run(None, {self.input_name: blob})

        boxes = boxes[0]
        conf = conf[0].reshape(-1)
        cls = cls[0].reshape(-1)

        best_idx = -1
        best_conf = self.conf_thres
        for i in range(len(conf)):
            if conf[i] < best_conf:
                continue
            if self.target_class is not None and int(cls[i]) != self.target_class:
                continue
            best_conf = conf[i]
            best_idx = i

        if best_idx < 0:
            return None

        x1, y1, x2, y2 = boxes[best_idx][:4]
        # Center in letterboxed coords, mapped back to original image coords.
        cx = ((x1 + x2) / 2.0 - self._left) / self._scale
        cy = ((y1 + y2) / 2.0 - self._top) / self._scale
        cx = float(np.clip(cx, 0, w - 1))
        cy = float(np.clip(cy, 0, h - 1))

        return np.array([cx, cy], dtype=np.float32)
