import os
from dotenv import load_dotenv

load_dotenv()

# Serial connection to the Arduino Mega
SERIAL_PORT = "/dev/ttyACM0"   # change to /dev/ttyUSB0 or COMx if needed
SERIAL_BAUD = 115200

# Seconds to wait for the Arduino to send ROVER:READY after connecting.
SERIAL_CONNECT_TIMEOUT = 10

# Seconds to wait for a command to complete (e.g. a long MOVE or PUMP run).
SERIAL_COMMAND_TIMEOUT = 120

# Roboflow — API key lives in .env (never commit that file)
ROBOFLOW_API_KEY  = os.environ.get("ROBOFLOW_API_KEY", "")
CLASSIFY_MODEL_ID = "plant_classification-2lw2t/1"
STEM_MODEL_ID     = "stem_detection-qeoi4/1"
