"""
Roboflow model wrappers (hosted API via inference-sdk).

Two models:
    Classifier   — image-level classification (plant type / health)
    StemDetector — object detection for plant stems

Both share a single InferenceHTTPClient and call the Roboflow serverless API.
API key is read from the ROBOFLOW_API_KEY environment variable (set in .env).

Usage:
    from makeathon.ml.models import Classifier, StemDetector

    clf   = Classifier()
    stems = StemDetector()

    result = clf.predict("data/2026_03_08_12_00_00.jpg")
    # {'class': 'weed', 'confidence': 0.91}

    result = stems.predict("data/2026_03_08_12_00_00.jpg")
    # {
    #   'detections':   [{'x': 320, 'y': 240, 'width': 40, 'height': 80,
    #                     'class': 'stem', 'confidence': 0.87}],
    #   'image_width':  640,
    #   'image_height': 480,
    # }
"""

from config.settings import ROBOFLOW_API_KEY, CLASSIFY_MODEL_ID, STEM_MODEL_ID

_API_URL = "https://serverless.roboflow.com"


class ModelError(Exception):
    """Raised when inference is misconfigured or fails."""


def _client():
    if not ROBOFLOW_API_KEY:
        raise ModelError("ROBOFLOW_API_KEY is not set — add it to your .env file.")
    from inference_sdk import InferenceHTTPClient
    return InferenceHTTPClient(api_url=_API_URL, api_key=ROBOFLOW_API_KEY)


class Classifier:
    """Wraps the Roboflow classification model."""

    def predict(self, image_path: str) -> dict:
        """
        Classify an image.

        Returns:
            {'class': str, 'confidence': float}
        """
        try:
            result = _client().infer(image_path, model_id=CLASSIFY_MODEL_ID)
            return {
                "class":      result["top"],
                "confidence": round(result["confidence"], 4),
            }
        except ModelError:
            raise
        except Exception as exc:
            raise ModelError(f"Classification failed: {exc}") from exc


class StemDetector:
    """Wraps the Roboflow stem detection model."""

    def predict(self, image_path: str) -> dict:
        """
        Detect stems in an image.

        Returns:
            {
                'detections':   list of {'x', 'y', 'width', 'height', 'class', 'confidence'},
                'image_width':  int,
                'image_height': int,
            }
        """
        try:
            result = _client().infer(image_path, model_id=STEM_MODEL_ID)
            detections = [
                {
                    "x":          p["x"],
                    "y":          p["y"],
                    "width":      p["width"],
                    "height":     p["height"],
                    "class":      p["class"],
                    "confidence": round(p["confidence"], 4),
                }
                for p in result.get("predictions", [])
            ]
            image = result.get("image", {})
            return {
                "detections":   detections,
                "image_width":  image.get("width",  640),
                "image_height": image.get("height", 480),
            }
        except ModelError:
            raise
        except Exception as exc:
            raise ModelError(f"Stem detection failed: {exc}") from exc
