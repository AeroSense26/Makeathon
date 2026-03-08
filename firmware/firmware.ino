/**
 * @file    firmware.ino
 * @brief   Autonomous Fertiliser Rover — Arduino Mega 2560 firmware entry point.
 *
 * This file is intentionally thin.  It owns only:
 *   • Hardware initialisation (setup)
 *   • The non-blocking main loop
 *   • Serial command parsing and dispatch
 *   • ACK/FINISH acknowledgement emission
 *
 * All hardware logic lives in actuators.h and sensors.h.
 * All configuration lives in config.h.
 *
 * ── Serial protocol (115 200 baud, LF or CR+LF terminated) ──────────────────
 *
 *   Command               Immediate ACK                   Completion ACK
 *   ─────────────────     ────────────────────────────    ─────────────────────
 *   READ_TEMP             TMP:<temp>,<hum>                —
 *   MOVE:FORWARD:NNNN     ACK:MOVE:FORWARD:NNNN           ACK:MOVE:FORWARD:FINISH
 *   MOVE:BACKWARD:NNNN    ACK:MOVE:BACKWARD:NNNN          ACK:MOVE:BACKWARD:FINISH
 *   SERVO:NNN             ACK:SERVO:NNN                   ACK:SERVO:FINISH
 *   PUMP:FORWARD:NN       ACK:PUMP:FORWARD:NN             ACK:PUMP:FINISH
 *   PUMP:BACKWARD:NN      ACK:PUMP:BACKWARD:NN            ACK:PUMP:FINISH
 *   STOP                  ACK:STOP                        —
 *
 *   On parse failure:     ERR:<reason>
 *
 * ── Required Arduino libraries ───────────────────────────────────────────────
 *   • AccelStepper       (Mike McCauley)  — Library Manager
 *   • Adafruit SHT4x     (Adafruit)       — Library Manager
 *   • Adafruit BusIO     (Adafruit)       — Library Manager (SHT4x dependency)
 *
 * @author  Rover Firmware Team
 * @date    2026-03-08
 */

#include "config.h"
#include "sensors.h"
#include "actuators.h"

// ─────────────────────────────────────────────────────────────────────────────
// Global controller instances
// ─────────────────────────────────────────────────────────────────────────────

static DrivetrainController drivetrain;
static ServoController      nozzle;
static PumpController       pump;
static SensorManager        sensors;

// ─────────────────────────────────────────────────────────────────────────────
// Active-command state
//
// Stored so that the completion ACKs can be formatted correctly without
// re-parsing the original command string.
// ─────────────────────────────────────────────────────────────────────────────

struct ActiveMoveState {
    char     direction[9];   // "FORWARD" or "BACKWARD"
    uint16_t steps;
    bool     active;
};

struct ActivePumpState {
    char    direction[9];
    uint8_t seconds;
    bool    active;
};

static ActiveMoveState s_move = { "", 0U, false };
static ActivePumpState s_pump = { "", 0U, false };

// ─────────────────────────────────────────────────────────────────────────────
// Serial receive buffer
// ─────────────────────────────────────────────────────────────────────────────

static char    s_rxBuf[SERIAL_BUFFER_SIZE];
static uint8_t s_rxIdx = 0U;

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations (defined below setup/loop for readability)
// ─────────────────────────────────────────────────────────────────────────────

static void processSerialInput();
static void dispatchCommand(const char* cmd);
static void cmdReadTemp();
static void cmdMove(const char* direction, uint16_t steps);
static void cmdServo(uint8_t position);
static void cmdPump(const char* direction, uint8_t seconds);
static void cmdStop();

// ═════════════════════════════════════════════════════════════════════════════
// Arduino entry points
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    // Give the host a moment to open its terminal before printing READY.
    delay(500);

    sensors.begin();    // Blocks briefly to prime the sensor ring-buffer.
    drivetrain.begin();
    nozzle.begin();
    pump.begin();

    Serial.println(F("ROVER:READY"));
}

void loop() {
    // ── 1. Update sensors (non-blocking rolling average) ─────────────────────
    sensors.update();

    // ── 2. Step motors and check for movement completion ─────────────────────
    drivetrain.update();
    if (s_move.active && drivetrain.justFinished()) {
        char finishMsg[40];
        snprintf(finishMsg, sizeof(finishMsg),
                 "ACK:MOVE:%s:FINISH", s_move.direction);
        Serial.println(finishMsg);
        s_move.active = false;
    }

    // ── 3. Check pump timer and emit completion ACK when done ────────────────
    pump.update();
    if (s_pump.active && pump.justFinished()) {
        Serial.println(F("ACK:PUMP:FINISH"));
        s_pump.active = false;
    }

    // ── 4. Parse incoming serial commands ────────────────────────────────────
    processSerialInput();
}

// ═════════════════════════════════════════════════════════════════════════════
// Serial input processing
// ═════════════════════════════════════════════════════════════════════════════

/**
 * @brief Drain the hardware UART receive buffer one byte at a time.
 *
 * Accumulates characters into s_rxBuf until a line terminator is seen, then
 * dispatches the null-terminated command string.  Handles both bare LF and
 * Windows-style CR+LF gracefully.  Silently discards characters that would
 * overflow the buffer to avoid undefined behaviour.
 */
static void processSerialInput() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (s_rxIdx > 0U) {
                s_rxBuf[s_rxIdx] = '\0';
                dispatchCommand(s_rxBuf);
                s_rxIdx = 0U;
            }
            // Ignore bare '\r' or empty lines.
        } else if (s_rxIdx < SERIAL_BUFFER_SIZE - 1U) {
            s_rxBuf[s_rxIdx++] = c;
        }
        // Overflow: silently drop the character.
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Command dispatcher
// ═════════════════════════════════════════════════════════════════════════════

/**
 * @brief Identify the command verb and route to the appropriate handler.
 *
 * Uses prefix matching with strncmp so no heap allocation is needed.
 * Unknown commands emit an ERR response instead of silently failing.
 *
 * @param cmd  Null-terminated, trimmed command string from the serial buffer.
 */
static void dispatchCommand(const char* cmd) {

    // ── READ_TEMP ─────────────────────────────────────────────────────────────
    if (strcmp(cmd, "READ_TEMP") == 0) {
        cmdReadTemp();
        return;
    }

    // ── STOP ──────────────────────────────────────────────────────────────────
    if (strcmp(cmd, "STOP") == 0) {
        cmdStop();
        return;
    }

    // ── MOVE:FORWARD/BACKWARD:NNNN ────────────────────────────────────────────
    if (strncmp(cmd, "MOVE:", 5) == 0) {
        char     direction[9] = { '\0' };
        uint16_t steps        = 0U;

        const char* rest  = cmd + 5;                  // Points past "MOVE:"
        const char* colon = strchr(rest, ':');

        if (colon == nullptr || (colon - rest) >= static_cast<int>(sizeof(direction))) {
            Serial.println(F("ERR:MOVE:PARSE_DIRECTION"));
            return;
        }

        strncpy(direction, rest, static_cast<size_t>(colon - rest));
        steps = static_cast<uint16_t>(atoi(colon + 1));

        if (strcmp(direction, "FORWARD") != 0 && strcmp(direction, "BACKWARD") != 0) {
            Serial.println(F("ERR:MOVE:INVALID_DIRECTION"));
            return;
        }

        cmdMove(direction, steps);
        return;
    }

    // ── SERVO:NNN ─────────────────────────────────────────────────────────────
    if (strncmp(cmd, "SERVO:", 6) == 0) {
        const int raw = atoi(cmd + 6);
        if (raw < 0 || raw > 100) {
            Serial.println(F("ERR:SERVO:RANGE_0_100"));
            return;
        }
        cmdServo(static_cast<uint8_t>(raw));
        return;
    }

    // ── PUMP:FORWARD/BACKWARD:NN ──────────────────────────────────────────────
    if (strncmp(cmd, "PUMP:", 5) == 0) {
        char    direction[9] = { '\0' };
        uint8_t seconds      = 0U;

        const char* rest  = cmd + 5;
        const char* colon = strchr(rest, ':');

        if (colon == nullptr || (colon - rest) >= static_cast<int>(sizeof(direction))) {
            Serial.println(F("ERR:PUMP:PARSE_DIRECTION"));
            return;
        }

        strncpy(direction, rest, static_cast<size_t>(colon - rest));

        const int rawSec = atoi(colon + 1);
        if (rawSec < 0 || rawSec > static_cast<int>(PUMP_MAX_DURATION_S)) {
            Serial.println(F("ERR:PUMP:DURATION_OUT_OF_RANGE"));
            return;
        }

        if (strcmp(direction, "FORWARD") != 0 && strcmp(direction, "BACKWARD") != 0) {
            Serial.println(F("ERR:PUMP:INVALID_DIRECTION"));
            return;
        }

        seconds = static_cast<uint8_t>(rawSec);
        cmdPump(direction, seconds);
        return;
    }

    // ── Unknown command ───────────────────────────────────────────────────────
    char errMsg[SERIAL_BUFFER_SIZE + 12];
    snprintf(errMsg, sizeof(errMsg), "ERR:UNKNOWN:%s", cmd);
    Serial.println(errMsg);
}

// ═════════════════════════════════════════════════════════════════════════════
// Command handlers
// ═════════════════════════════════════════════════════════════════════════════

/**
 * @brief Respond with the current rolling-average temperature and humidity.
 *
 * Response format:  TMP:23.4,56.7
 * Both values are formatted to one decimal place.
 */
static void cmdReadTemp() {
    float temperature = 0.0f;
    float humidity    = 0.0f;
    sensors.getAverages(temperature, humidity);

    char tempStr[8], humStr[8];
    dtostrf(temperature, 4, 1, tempStr);
    dtostrf(humidity,    4, 1, humStr);

    Serial.print(F("TMP:"));
    Serial.print(tempStr);
    Serial.print(F(","));
    Serial.println(humStr);
}

/**
 * @brief Begin a non-blocking movement and record state for FINISH ACK.
 *
 * The FINISH acknowledgement is emitted from loop() once drivetrain.justFinished()
 * returns true — it is not sent here.
 */
static void cmdMove(const char* direction, uint16_t steps) {
    // Record state for FINISH ACK emission in loop().
    strncpy(s_move.direction, direction, sizeof(s_move.direction) - 1U);
    s_move.direction[sizeof(s_move.direction) - 1U] = '\0';
    s_move.steps  = steps;
    s_move.active = true;

    char ack[32];
    snprintf(ack, sizeof(ack), "ACK:MOVE:%s:%04u", direction, steps);
    Serial.println(ack);

    drivetrain.move(direction, steps);
}

/**
 * @brief Command the servo to a position and immediately emit FINISH.
 *
 * The MG996R has no position feedback, so FINISH is sent once the PWM signal
 * has been written.  If downstream code must wait for physical travel, add a
 * fixed delay on the Raspberry Pi side.
 */
static void cmdServo(uint8_t position) {
    char ack[20];
    snprintf(ack, sizeof(ack), "ACK:SERVO:%03u", position);
    Serial.println(ack);

    nozzle.setPosition(position);

    Serial.println(F("ACK:SERVO:FINISH"));
}

/**
 * @brief Start a timed pump run and record state for FINISH ACK.
 *
 * The FINISH acknowledgement is emitted from loop() once pump.justFinished()
 * returns true — it is not sent here.
 */
static void cmdPump(const char* direction, uint8_t seconds) {
    // Record state for FINISH ACK emission in loop().
    strncpy(s_pump.direction, direction, sizeof(s_pump.direction) - 1U);
    s_pump.direction[sizeof(s_pump.direction) - 1U] = '\0';
    s_pump.seconds = seconds;
    s_pump.active  = true;

    char ack[32];
    snprintf(ack, sizeof(ack), "ACK:PUMP:%s:%02u", direction, seconds);
    Serial.println(ack);

    pump.run(direction, seconds);
}

/**
 * @brief Emergency-stop all actuators.
 *
 * Cancels any in-progress move or pump run.  The drivetrain decelerates
 * gracefully via AccelStepper's built-in ramp; the pump cuts immediately.
 */
static void cmdStop() {
    drivetrain.stop();
    pump.stop();

    // Cancel pending FINISH ACKs so they are not emitted after the stop.
    s_move.active = false;
    s_pump.active = false;

    Serial.println(F("ACK:STOP"));
}
