"""
Serial interface to the Arduino Mega rover firmware.

Provides ArduinoConnection, which opens the serial port, waits for the
firmware's ROVER:READY greeting, then exposes a single send() method that
transmits a command and blocks until the Arduino emits a terminal response.

Terminal responses (marks a command as complete):
    TMP:...            READ_TEMP reply
    ACK:STOP           STOP reply
    ACK:...:FINISH     end of any MOVE / SERVO / PUMP command
    ERR:...            firmware parse or range error
"""

import time
import serial


class ArduinoError(Exception):
    """Raised on connection failures, timeouts, or serial I/O errors."""


class ArduinoConnection:
    def __init__(self, port: str, baud: int, command_timeout: float = 120.0):
        """
        Args:
            port:            Serial device path, e.g. '/dev/ttyACM0'.
            baud:            Baud rate — must match the firmware (115200).
            command_timeout: Seconds to wait for a terminal response before
                             raising ArduinoError.
        """
        self._port = port
        self._baud = baud
        self._command_timeout = command_timeout
        self._serial: serial.Serial | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self, connect_timeout: float = 10.0) -> None:
        """Open the port and block until the firmware sends ROVER:READY."""
        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=1.0,        # readline() blocks for at most 1 s
            )
        except serial.SerialException as exc:
            raise ArduinoError(f"Could not open {self._port}: {exc}") from exc

        deadline = time.monotonic() + connect_timeout
        while time.monotonic() < deadline:
            line = self._readline()
            if line == "ROVER:READY":
                return

        raise ArduinoError(
            f"Timed out after {connect_timeout}s waiting for ROVER:READY. "
            "Check the port and baud rate in config/settings.py."
        )

    def disconnect(self) -> None:
        """Close the serial port if open."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    # ── Public command interface ──────────────────────────────────────────────

    def send(self, command: str) -> list[str]:
        """
        Transmit *command* and collect every response line until the Arduino
        signals completion.

        Args:
            command: Raw Arduino protocol string, e.g. 'MOVE:FORWARD:0200'.

        Returns:
            All response lines received up to and including the terminal line.

        Raises:
            ArduinoError: Not connected, serial I/O failure, or timeout.
        """
        if not self._serial or not self._serial.is_open:
            raise ArduinoError("Not connected to Arduino.")

        self._serial.write((command + "\n").encode("ascii"))
        self._serial.flush()

        responses: list[str] = []
        deadline = time.monotonic() + self._command_timeout

        while time.monotonic() < deadline:
            line = self._readline()
            if not line:
                continue
            responses.append(line)
            if self._is_terminal(line):
                return responses

        raise ArduinoError(
            f"Timed out after {self._command_timeout}s waiting for "
            f"response to '{command}'."
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _readline(self) -> str:
        """Read one line from the serial port. Returns '' on timeout."""
        if not self._serial:
            return ""
        try:
            raw = self._serial.readline()
            return raw.decode("ascii", errors="ignore").strip()
        except serial.SerialException as exc:
            raise ArduinoError(f"Serial read error: {exc}") from exc

    @staticmethod
    def _is_terminal(line: str) -> bool:
        """Return True if *line* marks the end of an Arduino command cycle."""
        return (
            line.startswith("TMP:")   # READ_TEMP response
            or line == "ACK:STOP"     # STOP response
            or line.endswith(":FINISH")  # MOVE / SERVO / PUMP completion
            or line.startswith("ERR:")   # firmware error
        )
