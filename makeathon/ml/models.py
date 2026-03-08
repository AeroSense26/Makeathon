"""
Roboflow model wrappers — plain HTTP, no heavy SDK required.

Calls the Roboflow serverless hosted API directly using `requests`.
No scipy, no supervision, no native builds needed — works on Pi out of the box.

API key is read from the ROBOFLOW_API_KEY environment variable (set in .env).

Usage:
    from makeathon.ml.models import Classifier, StemDetector

    result = Classifier().predict("data/2026_03_08_12_00_00.jpg")
    # {'class': 'weed', 'confidence': 0.91}

    result = StemDetector().predict("data/2026_03_08_12_00_00.jpg")
    # {
    #   'detections':   [{'x': 320, 'y': 240, 'width': 40, 'height': 80,
    #                     'class': 'stem', 'confidence': 0.87}],
    #   'image_width':  640,
    #   'image_height': 480,
    # }
"""

import base64
import requests

from config.settings import ROBOFLOW_API_KEY, CLASSIFY_MODEL_ID, STEM_MODEL_ID

_API_BASE = "https://serverless.roboflow.com"


class ModelError(Exception):
    """Raised when inference is misconfigured or fails."""


def _infer(model_id: str, image_path: str) -> dict:
    """POST an image to the Roboflow hosted API and return the parsed JSON."""
    if not ROBOFLOW_API_KEY:
        raise ModelError("ROBOFLOW_API_KEY is not set — add it to your .env file.")

    with open(image_path, "rb") as f:
        image_b64 = base64.b64encode(f.read()).decode("ascii")

    try:
        response = requests.post(
            f"{_API_BASE}/{model_id}",
            params={"api_key": ROBOFLOW_API_KEY},
            data=image_b64,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timeout=30,
        )
        response.raise_for_status()
        return response.json()
    except requests.RequestException as exc:
        raise ModelError(f"API request failed: {exc}") from exc


class Classifier:
    """Wraps the Roboflow classification model."""

    def predict(self, image_path: str) -> dict:
        """
        Classify an image.

        Returns:
            {'class': str, 'confidence': float}
        """
        try:
            result = _infer(CLASSIFY_MODEL_ID, image_path)
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
            result = _infer(STEM_MODEL_ID, image_path)
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
