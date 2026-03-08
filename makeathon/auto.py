"""
Autonomous stem-finding and centering loop.

Behaviour
---------
1.  Take a photo.
2.  Run stem detection.
3.  No stems found  →  drive FORWARD 8 000 steps and repeat from step 1.
4.  Stems found     →  compute the confidence-weighted average Y position.
        • The camera faces forward; a stem in the *upper* half of the frame
          is farther away  →  drive FORWARD 100 steps to close the distance.
        • A stem in the *lower* half is very close / already under the rover
          →  drive BACKWARD 100 steps.
        • Within ±20 % of the vertical centre  →  aligned, stop.

The loop runs until the stem is centred or an error/Ctrl-C occurs.
"""

from makeathon.hardware.arduino import ArduinoConnection, ArduinoError
from makeathon.hardware.camera import Camera, CameraError
from makeathon.ml.models import StemDetector, ModelError

SEARCH_STEPS    = 8000   # steps to advance when no stem is visible
CENTER_STEPS    = 100    # steps per fine-adjustment iteration
TOLERANCE       = 0.20   # accept when avg_y is within ±20 % of image centre


def _weighted_avg_y(detections: list) -> float:
    total_conf = sum(d["confidence"] for d in detections)
    return sum(d["y"] * d["confidence"] for d in detections) / total_conf


def _send(arduino: ArduinoConnection, cmd: str) -> None:
    responses = arduino.send(cmd)
    for line in responses:
        print(f"  {line}")


def run(arduino: ArduinoConnection, camera: Camera) -> None:
    """
    Entry point called by the CLI when the user types START.
    Blocks until the stem is centred, an error occurs, or Ctrl-C is pressed.
    """
    detector = StemDetector()
    print("Auto mode started. Press Ctrl-C to abort.\n")

    while True:
        # ── 1. Capture ────────────────────────────────────────────────────────
        try:
            path = camera.take_photo()
            print(f"Photo saved: {path.name}")
        except CameraError as exc:
            print(f"Camera error: {exc}")
            return

        # ── 2. Detect ─────────────────────────────────────────────────────────
        try:
            result = detector.predict(str(path))
        except ModelError as exc:
            print(f"Model error: {exc}")
            return

        detections   = result["detections"]
        image_height = result["image_height"]
        center_y     = image_height / 2.0

        # ── 3. No stems → search forward ──────────────────────────────────────
        if not detections:
            print(f"No stems detected. Moving forward {SEARCH_STEPS} steps...")
            try:
                _send(arduino, f"MOVE:FORWARD:{SEARCH_STEPS}")
            except ArduinoError as exc:
                print(f"Arduino error: {exc}")
                return
            continue

        # ── 4. Stems found → fine-centre on Y axis ────────────────────────────
        avg_y     = _weighted_avg_y(detections)
        deviation = (avg_y - center_y) / center_y   # negative = above, positive = below

        print(
            f"Stems: {len(detections)}  |  "
            f"avg_y={avg_y:.1f}  centre={center_y:.1f}  "
            f"deviation={deviation*100:+.1f}%"
        )

        if abs(deviation) <= TOLERANCE:
            print("\nStem centred. Ready to fertilize.")
            return

        if avg_y < center_y:
            # Stem above centre → still ahead of rover → move forward
            print(f"Stem above centre. Moving forward {CENTER_STEPS} steps...")
            direction = "FORWARD"
        else:
            # Stem below centre → rover has passed it → move backward
            print(f"Stem below centre. Moving backward {CENTER_STEPS} steps...")
            direction = "BACKWARD"

        try:
            _send(arduino, f"MOVE:{direction}:{CENTER_STEPS:04d}")
        except ArduinoError as exc:
            print(f"Arduino error: {exc}")
            return
