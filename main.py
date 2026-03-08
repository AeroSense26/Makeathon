"""
AeroSense — entry point.

Run from the repo root:
    python main.py

Make sure the Arduino is connected and SERIAL_PORT in config/settings.py
points to the correct device (e.g. /dev/ttyACM0, /dev/ttyUSB0, or COMx).
"""

import sys

from config.settings import (
    SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_CONNECT_TIMEOUT,
    SERIAL_COMMAND_TIMEOUT,
)
from makeathon.hardware.arduino import ArduinoConnection, ArduinoError
from makeathon.interface.cli import CLI


def main() -> None:
    print(f"Connecting to Arduino on {SERIAL_PORT} at {SERIAL_BAUD} baud...")

    arduino = ArduinoConnection(
        port=SERIAL_PORT,
        baud=SERIAL_BAUD,
        command_timeout=SERIAL_COMMAND_TIMEOUT,
    )

    try:
        arduino.connect(connect_timeout=SERIAL_CONNECT_TIMEOUT)
        print("Connected.\n")
        CLI(arduino).run()
    except ArduinoError as exc:
        print(f"Fatal: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        arduino.disconnect()


if __name__ == "__main__":
    main()
