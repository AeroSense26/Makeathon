# Serial connection to the Arduino Mega
SERIAL_PORT = "/dev/ttyACM0"   # change to /dev/ttyUSB0 or COMx if needed
SERIAL_BAUD = 115200

# Seconds to wait for the Arduino to send ROVER:READY after connecting.
SERIAL_CONNECT_TIMEOUT = 10

# Seconds to wait for a command to complete (e.g. a long MOVE or PUMP run).
SERIAL_COMMAND_TIMEOUT = 120
