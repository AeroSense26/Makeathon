"""
Raspberry Pi camera interface for the rover.

Captures a vertically-flipped still image (camera is mounted upside-down),
waits for autofocus to settle, then saves it as a JPEG to the repo-level
data/ directory with a timestamp filename.
"""

import time
from datetime import datetime
from pathlib import Path

# Repo root = two levels above this file (makeathon/hardware/camera.py)
DATA_DIR = Path(__file__).resolve().parents[2] / "data"

FOCUS_SECONDS = 3


class CameraError(Exception):
    """Raised on camera open failures or capture errors."""


class Camera:
    def __init__(self):
        self._cam = None

    def open(self) -> None:
        """Initialise and start the camera."""
        try:
            from picamera2 import Picamera2
            from libcamera import Transform

            self._cam = Picamera2()
            config = self._cam.create_still_configuration(
                transform=Transform(vflip=True)
            )
            self._cam.configure(config)
            self._cam.start()
        except Exception as exc:
            raise CameraError(f"Could not open camera: {exc}") from exc

    def close(self) -> None:
        """Stop and release the camera."""
        if self._cam:
            try:
                self._cam.stop()
                self._cam.close()
            except Exception:
                pass
            self._cam = None

    def take_photo(self) -> Path:
        """
        Wait for autofocus, capture a still, save to data/, return the path.

        Returns:
            pathlib.Path of the saved JPEG.

        Raises:
            CameraError: camera not open, or capture fails.
        """
        if not self._cam:
            raise CameraError("Camera is not open.")

        DATA_DIR.mkdir(parents=True, exist_ok=True)

        print(f"Focusing ({FOCUS_SECONDS}s)...")
        time.sleep(FOCUS_SECONDS)

        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        path = DATA_DIR / f"{timestamp}.jpg"

        try:
            self._cam.capture_file(str(path))
        except Exception as exc:
            raise CameraError(f"Capture failed: {exc}") from exc

        return path
