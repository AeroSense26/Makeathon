# AeroSense — Autonomous Precision Fertilizer Rover

> An intelligent, plant-aware rover that autonomously navigates crop rows, identifies individual plant stems via computer vision, classifies species, and dispenses a machine-learning-optimised dose of liquid fertilizer — all without human intervention.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [System Architecture](#system-architecture)
- [Autonomous Control Loop](#autonomous-control-loop)
- [Software Stack](#software-stack)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Configuration](#configuration)
- [CLI Reference](#cli-reference)
- [Serial Protocol](#serial-protocol)

---

## Overview

AeroSense is a precision agriculture rover built around an Arduino Mega 2560 and a Raspberry Pi 4B. It drives itself along a planting row, uses a side-mounted IMX708 camera and two Roboflow computer vision models to locate and identify plants, then applies a data-driven fertilizer dose predicted by an XGBoost regression model trained on environmental and morphological features.

**Key capabilities:**

| Capability | Implementation |
|---|---|
| Autonomous row navigation | Non-blocking stepper control via DRV8825 drivers |
| Line following | Dual TCRT5000 IR sensors with differential speed correction |
| Plant stem detection | Roboflow YOLOv8 instance segmentation (hosted API) |
| Species classification | Roboflow classification model — money tree, basil, anthurium |
| Fertilizer dosage | XGBoost regression on time, temperature, humidity, VPD, stem geometry |
| Environmental sensing | SEN0546 temperature / humidity sensor (rolling 10-point average) |
| Liquid delivery | DC pump via L298N driver, bidirectional (dispense + retract) |
| Nozzle aiming | MG996R servo — 0 (full left) to 100 (full right) |
| Emergency stop | Immediate motor + pump cutoff via serial or Ctrl-C |

---

## Hardware

### Compute

| Component | Role |
|---|---|
| Raspberry Pi 4B | Autonomy controller — vision, ML inference, CLI, serial comms |
| Arduino Mega 2560 + RAMPS 1.4 | Real-time firmware — steppers, servo, pump, sensors |

### Drivetrain

Four stepper motors (200 step/rev, 1.8°) driven by DRV8825 carriers in a fixed-axis tank configuration. The rover moves only forward and backward; lateral alignment is achieved by the IR line-following system.

| Motor | RAMPS slot | Physical position |
|---|---|---|
| FL | E0 | Front-left |
| FR | X | Front-right (DIR inverted) |
| BR | Y | Back-right (DIR inverted) |
| BL | Z | Back-left |

FR and BR are physically mirrored, so their direction pins are driven opposite to FL/BL to achieve straight-line motion.

### Sensors & Actuators

| Component | Connection | Purpose |
|---|---|---|
| TCRT5000 × 2 | D16 (left), D17 (right) | IR line following — black-tape track |
| SEN0546 | SDA/SCL (D20/D21) | Temperature & humidity (I²C) |
| IMX708 | Pi Camera connector | Plant imaging (mounted upside-down, left-facing) |
| MG996R servo | D11 (OC1A — Timer1 PWM) | Nozzle left/right aiming |
| DC water pump | D23 / D25 via L298N | Fertilizer dispense and retract |

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 4B                    │
│                                                     │
│  ┌──────────┐   ┌──────────┐   ┌─────────────────┐ │
│  │  Camera  │   │Roboflow  │   │  XGBRegressor   │ │
│  │ (IMX708) │──▶│  Models  │──▶│  Dosage Model   │ │
│  └──────────┘   └──────────┘   └────────┬────────┘ │
│                                          │          │
│  ┌───────────────────────────────────────▼────────┐ │
│  │              Control Loop (auto.py)            │ │
│  │  search → detect → centre → classify → dose   │ │
│  └───────────────────────────────────────┬────────┘ │
│                                          │ serial   │
└──────────────────────────────────────────┼──────────┘
                                           │ USB
┌──────────────────────────────────────────▼──────────┐
│              Arduino Mega 2560 + RAMPS 1.4          │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │Steppers  │  │  Servo   │  │   Water Pump     │  │
│  │(DRV8825) │  │ (Timer1) │  │   (L298N)        │  │
│  └──────────┘  └──────────┘  └──────────────────┘  │
│                                                     │
│  ┌──────────────────┐   ┌───────────────────────┐   │
│  │  IR Sensors ×2   │   │  SEN0546 Temp/Humid   │   │
│  │  (line follow)   │   │  (rolling 10s avg)    │   │
│  └──────────────────┘   └───────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

---

## Autonomous Control Loop

```
START
  │
  ▼
Move forward 8 000 steps
  │
  ▼
Take photo ──────────────────────────────────────┐
  │                                              │
  ▼                                              │
Stem detection (Roboflow YOLO)                   │
  │                                              │
  ├── No stems detected ────────────────────────►┘ (advance again)
  │
  ▼
Compute confidence-weighted average X position of stems
  │
  ├── |deviation| > 20% of image centre
  │     ├── Stem right of centre → MOVE:FORWARD:0100
  │     └── Stem left of centre  → MOVE:BACKWARD:0100
  │     └── (re-photograph and re-evaluate)
  │
  ▼ (centred)
Classify species (Roboflow classification model)
  │
  ▼
Gather environmental inputs:
  • Time of day
  • Temperature & humidity (SEN0546 rolling average)
  • VPD (calculated from T & RH)
  • Average stem width per pixel row (from segmentation mask)
  │
  ▼
XGBRegressor → predicted fertilizer duration (seconds)
  │
  ▼
SERVO:050  (deploy nozzle)
  │
  ▼
PUMP:FORWARD:<predicted_seconds>
  │
  ▼
PUMP:BACKWARD:20  (retract / clear line)
  │
  ▼
SERVO:100  (stow nozzle)
  │
  └──────────────────────────────────────────────┐
                                                 │
                                              (repeat)
```

---

## Software Stack

### Raspberry Pi (`python main.py`)

| Module | Purpose |
|---|---|
| `main.py` | Entry point — connects to Arduino, homes servo, launches CLI |
| `makeathon/interface/cli.py` | Interactive REPL — all commands, START/STOP handling |
| `makeathon/auto.py` | Autonomous fertilization loop |
| `makeathon/hardware/arduino.py` | Serial interface — `send()`, `connect()`, terminal detection |
| `makeathon/hardware/camera.py` | IMX708 capture — autofocus, vflip, timestamped JPEG to `data/` |
| `makeathon/ml/models.py` | Roboflow REST wrappers — `Classifier`, `StemDetector` |
| `config/settings.py` | Serial port, baud rate, model IDs — no secrets |
| `.env` | `ROBOFLOW_API_KEY` — gitignored, created per-machine |

### Arduino (`firmware/firmware.ino`)

Single-file firmware, zero external libraries.

| Feature | Implementation |
|---|---|
| Stepper control | Non-blocking `micros()` interval timer, 400 steps/s |
| Servo PWM | Timer1 Fast PWM Mode 14 (50 Hz) — no `Servo.h` needed |
| Pump control | Non-blocking `millis()` timer, bidirectional via L298N |
| IR line following | Differential speed correction on left/right DRV8825 enable pins |
| Temp/humidity | Raw I²C SEN0546 driver, rolling 10-point average over 10 s |
| Serial protocol | 115 200 baud, LF-terminated, ACK/FINISH handshake |

---

## Project Structure

```
Makeathon/
├── main.py                        # Entry point
├── requirements.txt
├── .env                           # API key — NOT committed
├── .gitignore
│
├── config/
│   └── settings.py                # Serial + Roboflow config
│
├── firmware/
│   └── firmware.ino               # Arduino firmware (no external libs)
│
├── makeathon/
│   ├── auto.py                    # Autonomous control loop
│   ├── hardware/
│   │   ├── arduino.py             # Serial interface class
│   │   └── camera.py              # Camera capture class
│   ├── interface/
│   │   └── cli.py                 # Interactive CLI
│   └── ml/
│       └── models.py              # Roboflow model wrappers
│
└── data/                          # Captured images (gitignored)
```

---

## Installation

### Arduino

1. Open `firmware/firmware.ino` in the Arduino IDE.
2. Select **Board → Arduino Mega or Mega 2560** and the correct port.
3. Upload. The serial monitor at 115 200 baud should print `ROVER:READY` on boot.

### Raspberry Pi

```bash
# Clone the repo
git clone <repo-url> ~/Makeathon
cd ~/Makeathon

# Create a virtual environment (system-site-packages needed for picamera2)
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# Install dependencies
pip install requests python-dotenv

# Create your secrets file
echo "ROBOFLOW_API_KEY=your_key_here" > .env

# Run
python main.py
```

---

## Configuration

All non-secret configuration lives in [config/settings.py](config/settings.py):

```python
SERIAL_PORT            = "/dev/ttyACM0"   # USB port for the Arduino
SERIAL_BAUD            = 115200
SERIAL_CONNECT_TIMEOUT = 10               # seconds to wait for ROVER:READY
SERIAL_COMMAND_TIMEOUT = 120              # seconds to wait for a command to finish

CLASSIFY_MODEL_ID      = "plant_classification-2lw2t/1"
STEM_MODEL_ID          = "stem_detection-qeoi4/1"
```

The `ROBOFLOW_API_KEY` is loaded from `.env` and never committed to source control.

Autonomous loop tuning constants are in [makeathon/auto.py](makeathon/auto.py):

```python
SEARCH_STEPS     = 8000   # steps forward between search frames
CENTER_STEPS     = 100    # steps per fine-alignment nudge
TOLERANCE        = 0.20   # ±20 % of image width counts as centred
DISPENSE_SECONDS = 25     # placeholder — replaced by XGBRegressor output
RETRACT_SECONDS  = 20     # pump reverse to clear the line
```

---

## CLI Reference

Run `python main.py` from the repo root. The Arduino must be connected and powered.

| Command | Description |
|---|---|
| `START` | Begin the autonomous fertilization loop |
| `STOP` | Emergency stop — immediately halts motors and pump |
| `TAKE_PHOTO` | Capture a timestamped image to `data/` |
| `READ_TEMP` | Request current temperature and humidity from Arduino |
| `MOVE:FORWARD:<steps>` | Drive all wheels forward e.g. `MOVE:FORWARD:0200` |
| `MOVE:BACKWARD:<steps>` | Drive all wheels backward |
| `SERVO:<0-100>` | Set nozzle position (0 = full left, 100 = full right) |
| `PUMP:FORWARD:<sec>` | Run pump forward for N seconds |
| `PUMP:BACKWARD:<sec>` | Run pump backward (retract) for N seconds |
| `help` | Print command reference |
| `exit` / `quit` | Disconnect and exit |

> **Ctrl-C at any time** sends an emergency STOP to the Arduino before exiting.

---

## Serial Protocol

All messages are ASCII, LF-terminated, at 115 200 baud.

### Commands (Pi → Arduino)

```
READ_TEMP
MOVE:FORWARD:0200
MOVE:BACKWARD:0200
SERVO:050
PUMP:FORWARD:25
PUMP:BACKWARD:20
STOP
```

### Responses (Arduino → Pi)

| Response | Meaning |
|---|---|
| `ROVER:READY` | Firmware booted, ready to accept commands |
| `TMP:23.4,61.2` | Temperature (°C), relative humidity (%) |
| `ACK:MOVE:FORWARD:0200` | Move command acknowledged |
| `ACK:MOVE:FORWARD:FINISH` | Move complete |
| `ACK:SERVO:050` | Servo command acknowledged |
| `ACK:SERVO:FINISH` | Servo reached position |
| `ACK:PUMP:FORWARD:25` | Pump command acknowledged |
| `ACK:PUMP:FINISH` | Pump timer elapsed |
| `ACK:STOP` | Emergency stop executed |
| `ERR:<reason>` | Parse or range error |

A command is considered complete when the Pi receives a line starting with `TMP:`, equal to `ACK:STOP`, ending with `:FINISH`, or starting with `ERR:`.
