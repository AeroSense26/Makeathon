"""
Autonomous fertilization loop.

Camera orientation
------------------
The camera is mounted on the LEFT side of the rover, facing LEFT.
As the rover moves FORWARD, plants travel RIGHT → LEFT across the image
(decreasing X).  Centering is therefore done on the horizontal (X) axis:

    Plant right of centre  →  rover hasn't reached it yet  →  FORWARD
    Plant left of centre   →  rover has passed it          →  BACKWARD
    Within ±20 % of centre →  aligned, proceed to fertilize

Full cycle (repeats until STOP / Ctrl-C)
-----------------------------------------
1.  Move FORWARD 8 000 steps and take a photo.
2.  Detect stems.  If none found, repeat step 1.
3.  Fine-adjust in 100-step increments until stem is X-centred.
4.  Move servo to position 50 (nozzle down).
5.  Pump FORWARD 25 s  (placeholder — will be XGBRegressor output).
6.  Pump BACKWARD 20 s (retract).
7.  Move servo to position 100 (nozzle up / home).
8.  Go to step 1 (search for the next plant).
"""

from makeathon.hardware.arduino import ArduinoConnection, ArduinoError
from makeathon.hardware.camera import Camera, CameraError
from makeathon.ml.models import StemDetector, ModelError

SEARCH_STEPS      = 8000   # steps forward when no stem visible
CENTER_STEPS      = 100    # steps per fine-adjustment iteration
TOLERANCE         = 0.20   # ±20 % of image centre counts as aligned

DISPENSE_SECONDS  = 25     # placeholder — replace with XGBRegressor output
RETRACT_SECONDS   = 20     # pump reverse to clear the line


def _weighted_avg_x(detections: list) -> float:
    total_conf = sum(d["confidence"] for d in detections)
    return sum(d["x"] * d["confidence"] for d in detections) / total_conf


def _send(arduino: ArduinoConnection, cmd: str) -> None:
    responses = arduino.send(cmd)
    for line in responses:
        print(f"  {line}")


def _search_forward(arduino: ArduinoConnection) -> None:
    print(f"No stems detected. Moving forward {SEARCH_STEPS} steps...")
    _send(arduino, f"MOVE:FORWARD:{SEARCH_STEPS}")


def _center_stem(arduino: ArduinoConnection, camera: Camera, detector: StemDetector) -> bool:
    """
    Take photos and nudge forward/backward until the stem is X-centred.

    Returns True when centred, False on any error.
    """
    while True:
        try:
            path = camera.take_photo()
            print(f"Photo: {path.name}")
        except CameraError as exc:
            print(f"Camera error: {exc}")
            return False

        try:
            result = detector.predict(str(path))
        except ModelError as exc:
            print(f"Model error: {exc}")
            return False

        detections  = result["detections"]
        image_width = result["image_width"]
        center_x    = image_width / 2.0

        if not detections:
            # Stem disappeared — go back to search
            print("Stem lost during centering. Resuming search...")
            return False

        avg_x     = _weighted_avg_x(detections)
        deviation = (avg_x - center_x) / center_x   # +ve = right of centre, -ve = left

        print(
            f"Stems: {len(detections)}  |  "
            f"avg_x={avg_x:.1f}  centre={center_x:.1f}  "
            f"deviation={deviation*100:+.1f}%"
        )

        if abs(deviation) <= TOLERANCE:
            print("Stem centred.")
            return True

        if avg_x > center_x:
            # Plant still to the right — rover hasn't reached it yet
            print(f"Stem right of centre. Moving forward {CENTER_STEPS} steps...")
            direction = "FORWARD"
        else:
            # Plant slid past centre to the left — back up
            print(f"Stem left of centre. Moving backward {CENTER_STEPS} steps...")
            direction = "BACKWARD"

        try:
            _send(arduino, f"MOVE:{direction}:{CENTER_STEPS:04d}")
        except ArduinoError as exc:
            print(f"Arduino error: {exc}")
            return False


def _fertilize(arduino: ArduinoConnection) -> None:
    """Deploy nozzle, dispense, retract, stow nozzle."""
    print("Deploying nozzle (servo → 50)...")
    _send(arduino, "SERVO:050")

    print(f"Dispensing water ({DISPENSE_SECONDS}s)...")
    _send(arduino, f"PUMP:FORWARD:{DISPENSE_SECONDS:02d}")

    print(f"Retracting ({RETRACT_SECONDS}s)...")
    _send(arduino, f"PUMP:BACKWARD:{RETRACT_SECONDS:02d}")

    print("Stowing nozzle (servo → 100)...")
    _send(arduino, "SERVO:100")


def run(arduino: ArduinoConnection, camera: Camera) -> None:
    """
    Entry point called by the CLI on START.
    Runs the full search → centre → fertilize cycle indefinitely
    until an error occurs or the user presses Ctrl-C / sends STOP.
    """
    detector = StemDetector()
    print("Auto mode started. Press Ctrl-C to abort.\n")

    while True:
        # ── Step 1: advance and capture ───────────────────────────────────────
        try:
            _search_forward(arduino)
        except ArduinoError as exc:
            print(f"Arduino error: {exc}")
            return

        try:
            path = camera.take_photo()
            print(f"Photo: {path.name}")
        except CameraError as exc:
            print(f"Camera error: {exc}")
            return

        # ── Step 2: detect ────────────────────────────────────────────────────
        try:
            result = detector.predict(str(path))
        except ModelError as exc:
            print(f"Model error: {exc}")
            return

        if not result["detections"]:
            continue   # nothing visible — loop back and advance again

        print(f"Stem(s) detected ({len(result['detections'])}).")

        # ── Step 3: centre ────────────────────────────────────────────────────
        centred = _center_stem(arduino, camera, detector)
        if not centred:
            continue   # lost the stem — go back to search

        # ── Steps 4-7: fertilize ──────────────────────────────────────────────
        try:
            _fertilize(arduino)
        except ArduinoError as exc:
            print(f"Arduino error during fertilization: {exc}")
            return

        print("Plant fertilized. Searching for next plant...\n")
