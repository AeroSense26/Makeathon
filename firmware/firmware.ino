/**
 * Rover firmware — minimal build (Arduino Mega 2560 / RAMPS 1.4)
 * No external libraries required.
 *
 * Commands (115 200 baud, LF-terminated):
 *   MOVE:FORWARD:NNNN    — drive all wheels forward NNNN steps
 *   MOVE:BACKWARD:NNNN   — drive all wheels backward NNNN steps
 *   SERVO:NNN            — set nozzle servo to position 0–100
 *   PUMP:FORWARD:NN      — run pump forward for NN seconds
 *   PUMP:BACKWARD:NN     — run pump backward for NN seconds
 *   STOP                 — immediately stop motors and pump
 *
 * Responses:
 *   ACK:MOVE:FORWARD:NNNN  →  (when done)  ACK:MOVE:FORWARD:FINISH
 *   ACK:SERVO:NNN          →               ACK:SERVO:FINISH
 *   ACK:PUMP:FORWARD:NN    →  (when done)  ACK:PUMP:FINISH
 *   ACK:STOP
 *   ERR:<reason>
 */

// ── Pin assignments ──────────────────────────────────────────────────────────
// RAMPS 1.4 stepper slots.  FR and BR are physically mirrored, so their
// DIR pins are driven opposite to FL/BL to achieve the same wheel direction.
//
//   [FL  E0]  [FR  X]
//   [BL   Z]  [BR  Y]

#define FL_STEP 26
#define FL_DIR  28
#define FL_EN   24

#define FR_STEP 54
#define FR_DIR  55
#define FR_EN   38

#define BR_STEP 60
#define BR_DIR  61
#define BR_EN   56

#define BL_STEP 46
#define BL_DIR  48
#define BL_EN   62

#define SERVO_PIN 11   // OC1A — Timer1 hardware PWM (no Servo.h needed)
#define PUMP_IN1  23   // L298N input 1
#define PUMP_IN2  25   // L298N input 2

// ── Tuning constants ─────────────────────────────────────────────────────────

#define STEP_INTERVAL_US 2500U   // 400 steps/s ≈ 2 RPM at 200 steps/rev
#define SERVO_MIN_US     1000U   // pulse width µs at position   0
#define SERVO_MAX_US     2000U   // pulse width µs at position 100
#define PUMP_MAX_SEC     60U     // safety ceiling for pump duration

// ── Runtime state ────────────────────────────────────────────────────────────

static long     g_stepsLeft  = 0L;      // +ve = forward, -ve = backward
static uint32_t g_lastStepUs = 0UL;
static bool     g_moveActive = false;   // true while waiting to emit FINISH
static char     g_moveDir[9] = "";      // "FORWARD" or "BACKWARD"

static uint32_t g_pumpEndMs  = 0UL;
static bool     g_pumpActive = false;

static char     g_rxBuf[48];
static uint8_t  g_rxIdx = 0U;

// ── Low-level helpers ────────────────────────────────────────────────────────

static void stepperDir(bool forward) {
    digitalWrite(FL_DIR, forward ? HIGH : LOW);
    digitalWrite(BL_DIR, forward ? HIGH : LOW);
    digitalWrite(FR_DIR, forward ? LOW  : HIGH);  // mirrored
    digitalWrite(BR_DIR, forward ? LOW  : HIGH);  // mirrored
    delayMicroseconds(5);                          // DRV8825 setup time
}

static void stepAll() {
    // Pulse all four STEP pins simultaneously.
    digitalWrite(FL_STEP, HIGH); digitalWrite(FR_STEP, HIGH);
    digitalWrite(BL_STEP, HIGH); digitalWrite(BR_STEP, HIGH);
    delayMicroseconds(2);        // DRV8825 min pulse width
    digitalWrite(FL_STEP, LOW);  digitalWrite(FR_STEP, LOW);
    digitalWrite(BL_STEP, LOW);  digitalWrite(BR_STEP, LOW);
}

static void pumpOff() {
    digitalWrite(PUMP_IN1, LOW);
    digitalWrite(PUMP_IN2, LOW);
}

static void servoSet(uint8_t pos) {
    pos = constrain(pos, 0, 100);
    uint16_t us = (uint16_t)map(pos, 0, 100, SERVO_MIN_US, SERVO_MAX_US);
    OCR1A = us * 2U;  // 2 Timer1 counts per µs (16 MHz / prescaler 8)
}

// ── Command dispatcher ───────────────────────────────────────────────────────

static void dispatch(const char* cmd) {

    // STOP ────────────────────────────────────────────────────────────────────
    if (strcmp(cmd, "STOP") == 0) {
        g_stepsLeft  = 0L;
        g_moveActive = false;
        pumpOff();
        g_pumpActive = false;
        Serial.println(F("ACK:STOP"));
        return;
    }

    // MOVE:FORWARD/BACKWARD:NNNN ──────────────────────────────────────────────
    if (strncmp(cmd, "MOVE:", 5) == 0) {
        char        dir[9] = {};
        const char* rest   = cmd + 5;
        const char* colon  = strchr(rest, ':');

        if (!colon || (colon - rest) >= 9) {
            Serial.println(F("ERR:MOVE:PARSE"));
            return;
        }
        strncpy(dir, rest, (size_t)(colon - rest));
        uint16_t steps = (uint16_t)atoi(colon + 1);

        if (strcmp(dir, "FORWARD") != 0 && strcmp(dir, "BACKWARD") != 0) {
            Serial.println(F("ERR:MOVE:DIR"));
            return;
        }

        bool fwd     = (strcmp(dir, "FORWARD") == 0);
        g_stepsLeft  = fwd ? (long)steps : -(long)steps;
        g_moveActive = true;
        g_lastStepUs = micros();
        strncpy(g_moveDir, dir, sizeof(g_moveDir) - 1);
        stepperDir(fwd);

        char ack[32];
        snprintf(ack, sizeof(ack), "ACK:MOVE:%s:%04u", dir, steps);
        Serial.println(ack);
        return;
    }

    // SERVO:NNN ───────────────────────────────────────────────────────────────
    if (strncmp(cmd, "SERVO:", 6) == 0) {
        int raw = atoi(cmd + 6);
        if (raw < 0 || raw > 100) {
            Serial.println(F("ERR:SERVO:RANGE_0_100"));
            return;
        }
        char ack[20];
        snprintf(ack, sizeof(ack), "ACK:SERVO:%03d", raw);
        Serial.println(ack);
        servoSet((uint8_t)raw);
        Serial.println(F("ACK:SERVO:FINISH"));
        return;
    }

    // PUMP:FORWARD/BACKWARD:NN ────────────────────────────────────────────────
    if (strncmp(cmd, "PUMP:", 5) == 0) {
        char        dir[9] = {};
        const char* rest   = cmd + 5;
        const char* colon  = strchr(rest, ':');

        if (!colon || (colon - rest) >= 9) {
            Serial.println(F("ERR:PUMP:PARSE"));
            return;
        }
        strncpy(dir, rest, (size_t)(colon - rest));
        int sec = atoi(colon + 1);

        if (strcmp(dir, "FORWARD") != 0 && strcmp(dir, "BACKWARD") != 0) {
            Serial.println(F("ERR:PUMP:DIR"));
            return;
        }
        if (sec < 0 || sec > (int)PUMP_MAX_SEC) {
            Serial.println(F("ERR:PUMP:DURATION"));
            return;
        }

        pumpOff();  // clean state before starting
        if (strcmp(dir, "FORWARD") == 0) {
            digitalWrite(PUMP_IN1, HIGH);
            digitalWrite(PUMP_IN2, LOW);
        } else {
            digitalWrite(PUMP_IN1, LOW);
            digitalWrite(PUMP_IN2, HIGH);
        }
        g_pumpEndMs  = millis() + (uint32_t)sec * 1000UL;
        g_pumpActive = true;

        char ack[32];
        snprintf(ack, sizeof(ack), "ACK:PUMP:%s:%02d", dir, sec);
        Serial.println(ack);
        return;
    }

    // Unknown ─────────────────────────────────────────────────────────────────
    char err[56];
    snprintf(err, sizeof(err), "ERR:UNKNOWN:%s", cmd);
    Serial.println(err);
}

// ── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // Steppers — all pins as outputs, drivers enabled (DRV8825 EN active-LOW)
    const uint8_t stepPins[] = { FL_STEP, FR_STEP, BR_STEP, BL_STEP };
    const uint8_t dirPins[]  = { FL_DIR,  FR_DIR,  BR_DIR,  BL_DIR  };
    const uint8_t enPins[]   = { FL_EN,   FR_EN,   BR_EN,   BL_EN   };
    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i],  OUTPUT);
        pinMode(enPins[i],   OUTPUT);
        digitalWrite(enPins[i], LOW);  // LOW = enabled on DRV8825
    }

    // Servo — Timer1 Fast PWM Mode 14 (ICR1 as TOP) at 50 Hz on pin 11 (OC1A)
    //   WGM13:0 = 1110  →  Fast PWM, TOP = ICR1
    //   COM1A1  = 1     →  Non-inverting output on OC1A
    //   CS11            →  Prescaler /8  →  1 tick = 0.5 µs at 16 MHz
    //   ICR1 = 39999    →  period = 40 000 × 0.5 µs = 20 ms (50 Hz)
    pinMode(SERVO_PIN, OUTPUT);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13)  | _BV(WGM12) | _BV(CS11);
    ICR1   = 39999U;
    servoSet(0);  // home position

    // Pump
    pinMode(PUMP_IN1, OUTPUT);
    pinMode(PUMP_IN2, OUTPUT);
    pumpOff();

    Serial.println(F("ROVER:READY"));
}

// ── Loop ─────────────────────────────────────────────────────────────────────

void loop() {

    // 1. Service steppers (non-blocking)
    if (g_stepsLeft != 0L) {
        const uint32_t now = micros();
        if ((uint32_t)(now - g_lastStepUs) >= STEP_INTERVAL_US) {
            g_lastStepUs = now;
            stepAll();
            if (g_stepsLeft > 0L) --g_stepsLeft;
            else                  ++g_stepsLeft;
        }
    } else if (g_moveActive) {
        // Steps just finished — emit FINISH once.
        char msg[40];
        snprintf(msg, sizeof(msg), "ACK:MOVE:%s:FINISH", g_moveDir);
        Serial.println(msg);
        g_moveActive = false;
    }

    // 2. Service pump timer (non-blocking)
    if (g_pumpActive && millis() >= g_pumpEndMs) {
        pumpOff();
        g_pumpActive = false;
        Serial.println(F("ACK:PUMP:FINISH"));
    }

    // 3. Drain and dispatch serial input
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_rxIdx > 0U) {
                g_rxBuf[g_rxIdx] = '\0';
                dispatch(g_rxBuf);
                g_rxIdx = 0U;
            }
        } else if (g_rxIdx < sizeof(g_rxBuf) - 1U) {
            g_rxBuf[g_rxIdx++] = c;
        }
    }
}
