"""
Roboflow model wrappers.

Two models are used:
    Classifier   — image-level classification (e.g. plant type / health)
    StemDetector — object detection for plant stems

Both are loaded lazily on first use so startup is fast even when the
models are large.  Model IDs and API key come from config/settings.py.

Usage:
    from makeathon.ml.models import Classifier, StemDetector

    clf  = Classifier()
    dets = StemDetector()

    result = clf.predict("data/2026_03_08_12_00_00.jpg")
    # {'class': 'weed', 'confidence': 0.91, ...}

    stems = dets.predict("data/2026_03_08_12_00_00.jpg")
    # [{'x': 320, 'y': 240, 'width': 40, 'height': 80,
    #   'class': 'stem', 'confidence': 0.87}, ...]
"""

from config.settings import ROBOFLOW_API_KEY, CLASSIFY_MODEL_ID, STEM_MODEL_ID


class ModelError(Exception):
    """Raised when a model cannot be loaded or inference fails."""


class Classifier:
    """Wraps the Roboflow classification model."""

    def __init__(self):
        self._model = None

    def _load(self) -> None:
        if self._model is not None:
            return
        if not ROBOFLOW_API_KEY or not CLASSIFY_MODEL_ID:
            raise ModelError(
                "ROBOFLOW_API_KEY and CLASSIFY_MODEL_ID must be set in config/settings.py"
            )
        try:
            from inference import get_model
            self._model = get_model(CLASSIFY_MODEL_ID, api_key=ROBOFLOW_API_KEY)
        except Exception as exc:
            raise ModelError(f"Could not load classifier: {exc}") from exc

    def predict(self, image_path: str) -> dict:
        """
        Run classification on an image file.

        Args:
            image_path: Path to a JPEG/PNG image.

        Returns:
            Dict with at least 'class' and 'confidence' keys.

        Raises:
            ModelError: model not configured or inference fails.
        """
        self._load()
        try:
            results = self._model.infer(image_path)
            # inference SDK returns a list; take the top prediction
            top = results[0].predictions[0]
            return {"class": top.class_name, "confidence": round(top.confidence, 4)}
        except Exception as exc:
            raise ModelError(f"Classification failed: {exc}") from exc


class StemDetector:
    """Wraps the Roboflow stem detection model."""

    def __init__(self):
        self._model = None

    def _load(self) -> None:
        if self._model is not None:
            return
        if not ROBOFLOW_API_KEY or not STEM_MODEL_ID:
            raise ModelError(
                "ROBOFLOW_API_KEY and STEM_MODEL_ID must be set in config/settings.py"
            )
        try:
            from inference import get_model
            self._model = get_model(STEM_MODEL_ID, api_key=ROBOFLOW_API_KEY)
        except Exception as exc:
            raise ModelError(f"Could not load stem detector: {exc}") from exc

    def predict(self, image_path: str) -> list:
        """
        Run stem detection on an image file.

        Args:
            image_path: Path to a JPEG/PNG image.

        Returns:
            List of dicts, each with 'x', 'y', 'width', 'height',
            'class', and 'confidence'.  Empty list if no stems found.

        Raises:
            ModelError: model not configured or inference fails.
        """
        self._load()
        try:
            results = self._model.infer(image_path)
            detections = []
            for pred in results[0].predictions:
                detections.append({
                    "x":          pred.x,
                    "y":          pred.y,
                    "width":      pred.width,
                    "height":     pred.height,
                    "class":      pred.class_name,
                    "confidence": round(pred.confidence, 4),
                })
            return detections
        except Exception as exc:
            raise ModelError(f"Stem detection failed: {exc}") from exc
