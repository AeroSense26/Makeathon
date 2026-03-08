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
1.  Move FORWARD 8 000 steps.
2.  Take a photo and run stem detection.
3.  No stems found → repeat from step 1.
4.  Fine-adjust in 100-step increments until the stem is X-centred (±20 %).
5.  Classify plant species.
6.  Read temperature and humidity; calculate VPD and stem geometry.
7.  XGBRegressor predicts optimal fertilizer duration.
8.  SERVO:050 → deploy nozzle.
9.  PUMP:FORWARD:<seconds> → dispense.
10. PUMP:BACKWARD:20 → retract / clear the line.
11. SERVO:100 → stow nozzle.
12. Go to step 1 for the next plant.
"""

from datetime import datetime
from pathlib import Path

from makeathon.hardware.arduino import ArduinoConnection, ArduinoError
from makeathon.hardware.camera import Camera, CameraError
from makeathon.ml.models import Classifier, StemDetector, ModelError
from makeathon.ml.regressor import FertilizerPredictor, vpd_kpa

# ── Tuning constants ──────────────────────────────────────────────────────────

SEARCH_STEPS    = 8000   # steps forward between search frames
CENTER_STEPS    = 100    # steps per fine-alignment nudge
TOLERANCE       = 0.20   # ±20 % of image width counts as centred
RETRACT_SECONDS = 20     # pump reverse duration to clear the line


# ── Low-level helpers ─────────────────────────────────────────────────────────

def _send(arduino: ArduinoConnection, cmd: str) -> list[str]:
    """Send a command, print all response lines, and return them."""
    responses = arduino.send(cmd)
    for line in responses:
        print(f"  {line}")
    return responses


def _weighted_avg_x(detections: list) -> float:
    total_conf = sum(d["confidence"] for d in detections)
    return sum(d["x"] * d["confidence"] for d in detections) / total_conf


def _get_temp_humidity(arduino: ArduinoConnection) -> tuple[float, float]:
    """
    Request temperature and humidity from the Arduino.

    Returns (temp_c, humidity_pct).  Falls back to safe defaults on any
    failure so the fertilization cycle is never blocked by a sensor issue.

    Note: the firmware currently returns a Fahrenheit placeholder (70.2 °F).
    Values above 50 are assumed to be Fahrenheit and are converted.  Once the
    SEN0546 is live the firmware will emit Celsius natively and this branch
    will never trigger.
    """
    try:
        responses = arduino.send("READ_TEMP")
        for line in responses:
            if line.startswith("TMP:"):
                parts = line[4:].split(",")
                temp  = float(parts[0])
                hum   = float(parts[1])
                if temp > 50.0:                     # Fahrenheit placeholder
                    temp = (temp - 32.0) * 5.0 / 9.0
                return round(temp, 1), round(hum, 1)
    except Exception:
        pass
    print("  Warning: temperature read failed — using defaults (22 °C, 60 %).")
    return 22.0, 60.0


# ── Phase helpers ─────────────────────────────────────────────────────────────

def _search_forward(arduino: ArduinoConnection) -> None:
    print(f"No stems detected. Moving forward {SEARCH_STEPS} steps...")
    _send(arduino, f"MOVE:FORWARD:{SEARCH_STEPS}")


def _center_stem(
    arduino: ArduinoConnection,
    camera: Camera,
    detector: StemDetector,
) -> tuple[bool, Path | None, list]:
    """
    Nudge the rover forward/backward in CENTER_STEPS increments until the
    confidence-weighted average X position of detected stems falls within
    TOLERANCE of the horizontal image centre.

    Returns
    -------
    (centred, last_path, last_detections)
        centred          : True if the stem was successfully centred
        last_path        : Path of the most recent photo (or None)
        last_detections  : Detection list from the centred frame (or [])
    """
    last_path:       Path | None = None
    last_detections: list        = []

    while True:
        # Capture
        try:
            path = camera.take_photo()
            last_path = path
            print(f"Photo: {path.name}")
        except CameraError as exc:
            print(f"Camera error: {exc}")
            return False, last_path, last_detections

        # Detect
        try:
            result = detector.predict(str(path))
        except ModelError as exc:
            print(f"Model error: {exc}")
            return False, last_path, last_detections

        detections  = result["detections"]
        image_width = result["image_width"]
        center_x    = image_width / 2.0

        if not detections:
            print("Stem lost during centering. Resuming search...")
            return False, last_path, last_detections

        last_detections = detections
        avg_x           = _weighted_avg_x(detections)
        deviation       = (avg_x - center_x) / center_x   # +ve = right, -ve = left

        print(
            f"Stems: {len(detections)}  |  "
            f"avg_x={avg_x:.1f}  centre={center_x:.1f}  "
            f"deviation={deviation * 100:+.1f}%"
        )

        if abs(deviation) <= TOLERANCE:
            print("Stem centred.")
            return True, path, detections

        direction = "FORWARD" if avg_x > center_x else "BACKWARD"
        label     = "right of" if avg_x > center_x else "left of"
        print(f"Stem {label} centre. Moving {direction} {CENTER_STEPS} steps...")

        try:
            _send(arduino, f"MOVE:{direction}:{CENTER_STEPS:04d}")
        except ArduinoError as exc:
            print(f"Arduino error: {exc}")
            return False, last_path, last_detections


def _fertilize(
    arduino:    ArduinoConnection,
    detections: list,
    species:    str,
) -> None:
    """
    Gather environmental inputs, predict the optimal dose via XGBRegressor,
    then execute the full dispense cycle.
    """
    predictor = FertilizerPredictor()

    # Environmental inputs
    temp_c, humidity = _get_temp_humidity(arduino)
    hour             = datetime.now().hour
    avg_stem_width   = sum(d["width"] for d in detections) / len(detections)
    vpd              = vpd_kpa(temp_c, humidity)

    # Predict
    seconds = predictor.predict(
        hour=hour,
        temp_c=temp_c,
        humidity_pct=humidity,
        avg_stem_width_px=avg_stem_width,
        species=species,
    )

    print(
        f"  Species: {species} | "
        f"Temp: {temp_c} °C | RH: {humidity} % | "
        f"VPD: {vpd:.2f} kPa | "
        f"Stem width: {avg_stem_width:.1f} px | "
        f"Hour: {hour:02d}:xx"
    )
    print(f"  XGBRegressor → dispense {seconds} s")

    # Dispense cycle
    print("Deploying nozzle (servo → 50)...")
    _send(arduino, "SERVO:050")

    print(f"Dispensing ({seconds} s)...")
    _send(arduino, f"PUMP:FORWARD:{int(round(seconds)):02d}")

    print(f"Retracting ({RETRACT_SECONDS} s)...")
    _send(arduino, f"PUMP:BACKWARD:{RETRACT_SECONDS:02d}")

    print("Stowing nozzle (servo → 100)...")
    _send(arduino, "SERVO:100")


# ── Entry point ───────────────────────────────────────────────────────────────

def run(arduino: ArduinoConnection, camera: Camera) -> None:
    """
    Called by the CLI on START.  Runs the full search → centre → classify →
    predict → fertilize cycle indefinitely until an error occurs or the user
    presses Ctrl-C.
    """
    detector   = StemDetector()
    classifier = Classifier()
    print("Auto mode started. Press Ctrl-C to abort.\n")

    while True:

        # ── 1. Advance and capture ────────────────────────────────────────────
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

        # ── 2. Detect ─────────────────────────────────────────────────────────
        try:
            result = detector.predict(str(path))
        except ModelError as exc:
            print(f"Model error: {exc}")
            return

        if not result["detections"]:
            continue

        print(f"Stem(s) detected ({len(result['detections'])}).")

        # ── 3. Centre ─────────────────────────────────────────────────────────
        centred, centred_path, centred_detections = _center_stem(
            arduino, camera, detector
        )
        if not centred:
            continue

        # ── 4. Classify ───────────────────────────────────────────────────────
        species = "basil"   # safe fallback
        if centred_path:
            try:
                cls = classifier.predict(str(centred_path))
                species = cls["class"]
                print(f"Classified: {species}  (confidence {cls['confidence']:.1%})")
            except ModelError as exc:
                print(f"Classification failed ({exc}) — defaulting to '{species}'.")

        # ── 5. Fertilize ──────────────────────────────────────────────────────
        try:
            _fertilize(arduino, centred_detections, species)
        except ArduinoError as exc:
            print(f"Arduino error during fertilization: {exc}")
            return

        print("Plant fertilized. Searching for next plant...\n")
