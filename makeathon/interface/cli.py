"""
Interactive command-line interface for the rover.

Accepts the same command syntax used by the Arduino firmware, so what you
type is exactly what gets sent over serial.  Responses are printed as
received.  TAKE_PHOTO is handled locally on the Pi.

Supported commands:
    READ_TEMP
    MOVE:FORWARD:<steps>     e.g. MOVE:FORWARD:0200
    MOVE:BACKWARD:<steps>
    SERVO:<position>         0-100
    PUMP:FORWARD:<seconds>   e.g. PUMP:FORWARD:05
    PUMP:BACKWARD:<seconds>
    STOP
    TAKE_PHOTO               focus then capture a timestamped JPEG to data/
    exit / quit              leave the CLI
"""

from makeathon.hardware.arduino import ArduinoConnection, ArduinoError
from makeathon.hardware.camera import Camera, CameraError

_EXACT_COMMANDS = {"READ_TEMP", "STOP", "TAKE_PHOTO"}
_PREFIX_COMMANDS = ("MOVE:", "SERVO:", "PUMP:")

_HELP = (
    "Commands:\n"
    "  READ_TEMP\n"
    "  MOVE:FORWARD:<steps>   |  MOVE:BACKWARD:<steps>\n"
    "  SERVO:<0-100>\n"
    "  PUMP:FORWARD:<sec>     |  PUMP:BACKWARD:<sec>\n"
    "  STOP\n"
    "  TAKE_PHOTO\n"
    "  help  |  exit"
)


def _is_valid(cmd: str) -> bool:
    return cmd in _EXACT_COMMANDS or any(cmd.startswith(p) for p in _PREFIX_COMMANDS)


class CLI:
    def __init__(self, arduino: ArduinoConnection):
        self._arduino = arduino
        self._camera = Camera()

    def run(self) -> None:
        """Start the read-eval-print loop. Exits on 'exit', 'quit', or Ctrl-C."""
        try:
            self._camera.open()
            print("Camera ready.")
        except CameraError as exc:
            print(f"Warning: camera unavailable — {exc}")
            print("TAKE_PHOTO will not work this session.")

        print("Rover ready.  Type 'help' for commands.\n")

        try:
            self._loop()
        finally:
            self._camera.close()

    def _loop(self) -> None:
        while True:
            try:
                raw = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                break

            if not raw:
                continue

            cmd = raw.upper()

            if cmd in ("EXIT", "QUIT"):
                print("Exiting.")
                break

            if cmd == "HELP":
                print(_HELP)
                continue

            if not _is_valid(cmd):
                print(f"Unknown command: '{raw}'.  Type 'help' to see available commands.")
                continue

            if cmd == "TAKE_PHOTO":
                try:
                    path = self._camera.take_photo()
                    print(f"Saved: {path}")
                except CameraError as exc:
                    print(f"Camera error: {exc}")
                continue

            try:
                responses = self._arduino.send(cmd)
                for line in responses:
                    print(line)
            except ArduinoError as exc:
                print(f"Error: {exc}")
