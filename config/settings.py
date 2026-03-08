# Serial connection to the Arduino Mega
SERIAL_PORT = "/dev/ttyACM0"   # change to /dev/ttyUSB0 or COMx if needed
SERIAL_BAUD = 115200

# Seconds to wait for the Arduino to send ROVER:READY after connecting.
SERIAL_CONNECT_TIMEOUT = 10

# Seconds to wait for a command to complete (e.g. a long MOVE or PUMP run).
SERIAL_COMMAND_TIMEOUT = 120

# Roboflow — fill in your API key and model IDs (workspace/project/version)
ROBOFLOW_API_KEY    = ""
CLASSIFY_MODEL_ID   = ""   # e.g. "my-workspace/plant-classifier/1"
STEM_MODEL_ID       = ""   # e.g. "my-workspace/stem-detector/1"
