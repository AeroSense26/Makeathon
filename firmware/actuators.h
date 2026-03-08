/**
 * @file    actuators.h
 * @brief   Drivetrain, servo nozzle, and pump controllers for the rover.
 *
 * Three independent controller classes are provided:
 *
 *   DrivetrainController
 *     Manages 4× SimpleStepper instances (tank-drive layout) and delegates
 *     to an IRSensorPair (from sensors.h) for live line-following correction.
 *     Movement commands are non-blocking — call update() every loop() tick.
 *
 *   ServoController
 *     Wraps the MG996R servo with a 0–100 positional API that maps to the
 *     calibrated physical angle range defined in config.h.
 *
 *   PumpController
 *     Drives the DC pump via an L298N motor driver for a caller-specified
 *     duration in seconds.  Timing is non-blocking — call update() every tick.
 *
 * No external libraries required.
 *   • SimpleStepper is implemented here — no AccelStepper needed.
 *   • Servo.h is bundled with the Arduino IDE (no Library Manager install).
 *   • IRSensorPair is in sensors.h (also no external dependency).
 *
 * @author  Rover Firmware Team
 * @date    2026-03-08
 */

#pragma once

#include <Arduino.h>
#include <Servo.h>      // Bundled with Arduino IDE — no install required.

#include "config.h"
#include "sensors.h"   // Brings in IRSensorPair.

// ═════════════════════════════════════════════════════════════════════════════
// SimpleStepper — non-blocking stepper driver with trapezoidal velocity profile
//
// Drop-in replacement for AccelStepper (DRIVER mode).  Mirrors the subset of
// the AccelStepper API used by DrivetrainController so the controller code
// needs no changes other than the type name.
// ═════════════════════════════════════════════════════════════════════════════

class SimpleStepper {
public:
    /**
     * @param stepPin    STEP output → DRV8825 STEP pin.
     * @param dirPin     DIR  output → DRV8825 DIR  pin.
     * @param enablePin  EN   output → DRV8825 EN   pin (active-LOW on DRV8825).
     */
    SimpleStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin)
        : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin) {}

    // ── Configuration ─────────────────────────────────────────────────────────

    /**
     * @brief Mirror AccelStepper::setPinsInverted.
     *
     * @param invertDir     Swap the logical meaning of HIGH/LOW on DIR.
     * @param invertStep    Unused — DRV8825 steps on the LOW→HIGH edge regardless.
     * @param invertEnable  When true, writing LOW to EN enables the driver
     *                      (DRV8825 default — RAMPS wires EN active-LOW).
     */
    void setPinsInverted(bool invertDir, bool /*invertStep*/, bool invertEnable) {
        _invertDir    = invertDir;
        _invertEnable = invertEnable;
    }

    /** @brief Set the top cruising speed in steps per second. */
    void setMaxSpeed(float sps) { _maxSpeed = max(sps, 1.0f); }

    /** @brief Set the acceleration / deceleration in steps per second². */
    void setAcceleration(float spss) { _acceleration = max(spss, 1.0f); }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @brief Configure all three pins as outputs and enable the DRV8825.
     * Call once from setup() after setPinsInverted / setMaxSpeed / setAcceleration.
     */
    void enableOutputs() {
        pinMode(_stepPin,   OUTPUT);
        pinMode(_dirPin,    OUTPUT);
        pinMode(_enablePin, OUTPUT);

        // DRV8825 EN is active-LOW; _invertEnable == true means "invert the
        // logic level", so LOW enables the driver when the flag is set.
        digitalWrite(_enablePin, _invertEnable ? LOW : HIGH);
    }

    // ── Movement Commands ─────────────────────────────────────────────────────

    /**
     * @brief Queue a relative movement.
     *
     * Positive delta = forward (as wired); negative = reverse.
     * Resets the velocity ramp and sets the direction pin immediately.
     * Has no effect when delta is zero.
     *
     * @param delta  Signed step count.
     */
    void move(long delta) {
        if (delta == 0L) return;

        _stepsLeft    = static_cast<uint32_t>(abs(delta));
        _forward      = (delta > 0L);
        _currentSpeed = _minSpeed();   // Begin ramp from near-zero.
        _justFinished = false;
        _lastStepUs   = micros();

        // Set direction — XOR with invert flag for physically mirrored motors.
        digitalWrite(_dirPin, (_forward != _invertDir) ? HIGH : LOW);
        delayMicroseconds(5U);  // DRV8825 DIR setup time (spec: ≥ 650 ns).
    }

    /**
     * @brief Service the stepper — must be called every loop() iteration.
     *
     * Generates one STEP pulse if the inter-step interval has elapsed, then
     * updates the running speed according to the trapezoidal velocity profile:
     *   • Ramp up  until _maxSpeed is reached.
     *   • Cruise   at _maxSpeed.
     *   • Ramp down when remaining steps ≤ steps needed to stop cleanly.
     *
     * @return true while still in motion; false when the target is reached.
     */
    bool run() {
        if (_stepsLeft == 0U) return false;

        const uint32_t now      = micros();
        const uint32_t interval = static_cast<uint32_t>(1000000.0f / _currentSpeed);

        // Cast to uint32_t preserves correctness across micros() overflow (~70 min).
        if (static_cast<uint32_t>(now - _lastStepUs) < interval) return true;
        _lastStepUs = now;

        // STEP pulse — DRV8825 minimum high/low pulse width: 1.9 µs.
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(2U);
        digitalWrite(_stepPin, LOW);

        if (--_stepsLeft == 0U) {
            _currentSpeed = 0.0f;
            _justFinished = true;
            return false;
        }

        // ── Trapezoidal ramp ──────────────────────────────────────────────────
        // Δv per step = a × Δt = a / v   (Δt between steps = 1/v seconds).
        const float stepsToStop =
            (_currentSpeed * _currentSpeed) / (2.0f * _acceleration);

        if (static_cast<float>(_stepsLeft) <= stepsToStop) {
            // Deceleration phase — ramp down, but never below the start speed.
            _currentSpeed -= _acceleration / _currentSpeed;
            if (_currentSpeed < _minSpeed()) _currentSpeed = _minSpeed();
        } else {
            // Acceleration phase — ramp up toward the configured max speed.
            _currentSpeed += _acceleration / _currentSpeed;
            if (_currentSpeed > _maxSpeed) _currentSpeed = _maxSpeed;
        }

        return true;
    }

    /**
     * @brief Immediately halt step generation.
     *
     * DrivetrainController sets its own _moving flag to false immediately after
     * calling stop(), so run() is never called again.  Zeroing _stepsLeft is a
     * safety measure in case the call sequence changes in future.
     */
    void stop() {
        _stepsLeft    = 0U;
        _currentSpeed = 0.0f;
        _justFinished = false;
    }

    // ── State Queries ─────────────────────────────────────────────────────────

    /** @brief True while step generation is active. */
    bool isRunning() const { return _stepsLeft > 0U; }

    /**
     * @brief Consumes and returns the one-shot "just finished" flag.
     *
     * Returns true exactly once after a move() command completes naturally.
     * The flag is cleared by this call, matching AccelStepper's semantics.
     */
    bool justFinished() {
        if (_justFinished) { _justFinished = false; return true; }
        return false;
    }

private:
    /**
     * @brief Minimum starting speed derived from first-step kinematics.
     *
     * For a body starting from rest: v = √(2 · a · Δx), where Δx = 1 step.
     * This prevents a divide-by-zero on the first ramp iteration.
     */
    float _minSpeed() const { return sqrtf(2.0f * _acceleration); }

    // Pin assignments
    uint8_t  _stepPin;
    uint8_t  _dirPin;
    uint8_t  _enablePin;

    // Pin inversion flags
    bool     _invertDir    = false;
    bool     _invertEnable = false;

    // Velocity profile state
    float    _maxSpeed     = 100.0f;
    float    _acceleration = 100.0f;
    float    _currentSpeed = 0.0f;

    // Step-count and timing state
    uint32_t _stepsLeft    = 0U;
    bool     _forward      = true;
    uint32_t _lastStepUs   = 0UL;
    bool     _justFinished = false;
};

// ═════════════════════════════════════════════════════════════════════════════
// DrivetrainController
// ═════════════════════════════════════════════════════════════════════════════

class DrivetrainController {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Initialise all four steppers and the IR sensor pair.
     * Call once from setup().
     */
    void begin() {
        // Configure steppers.  setPinsInverted third arg (true) = EN active-LOW.
        _fl.setMaxSpeed(MOTOR_MAX_SPEED_SPS);
        _fl.setAcceleration(MOTOR_ACCELERATION_SPSS);
        _fl.setPinsInverted(MOTOR_FL_INVERT_DIR, false, true);
        _fl.enableOutputs();

        _fr.setMaxSpeed(MOTOR_MAX_SPEED_SPS);
        _fr.setAcceleration(MOTOR_ACCELERATION_SPSS);
        _fr.setPinsInverted(MOTOR_FR_INVERT_DIR, false, true);
        _fr.enableOutputs();

        _br.setMaxSpeed(MOTOR_MAX_SPEED_SPS);
        _br.setAcceleration(MOTOR_ACCELERATION_SPSS);
        _br.setPinsInverted(MOTOR_BR_INVERT_DIR, false, true);
        _br.enableOutputs();

        _bl.setMaxSpeed(MOTOR_MAX_SPEED_SPS);
        _bl.setAcceleration(MOTOR_ACCELERATION_SPSS);
        _bl.setPinsInverted(MOTOR_BL_INVERT_DIR, false, true);
        _bl.enableOutputs();

        // IR sensors — ownership of pin setup lives in IRSensorPair.
        _ir.begin();

        Serial.println(F("DRIVETRAIN:READY"));
    }

    // ── Non-blocking Update ───────────────────────────────────────────────────

    /**
     * @brief Must be called every iteration of loop().
     *
     * Steps each motor toward its target position and applies live IR speed
     * correction.  Returns immediately; never blocks.
     */
    void update() {
        if (!_moving) return;

        _applyIRCorrection();

        const bool flRunning = _fl.run();
        const bool frRunning = _fr.run();
        const bool brRunning = _br.run();
        const bool blRunning = _bl.run();

        if (!flRunning && !frRunning && !brRunning && !blRunning) {
            _moving       = false;
            _justFinished = true;
            _setAllSpeeds(MOTOR_MAX_SPEED_SPS);  // Restore for next command.
        }
    }

    // ── Command Interface ─────────────────────────────────────────────────────

    /**
     * @brief Queue a relative movement command.
     *
     * @param direction  "FORWARD" or "BACKWARD" (case-sensitive).
     * @param steps      Number of steps to travel (direction sets sign).
     */
    void move(const char* direction, uint16_t steps) {
        const long delta = (strcmp(direction, "FORWARD") == 0)
                               ? static_cast<long>(steps)
                               : -static_cast<long>(steps);

        _fl.move(delta);
        _fr.move(delta);
        _br.move(delta);
        _bl.move(delta);

        _setAllSpeeds(MOTOR_MAX_SPEED_SPS);

        _moving       = true;
        _justFinished = false;
    }

    /**
     * @brief Immediately halt all motors.
     *
     * Sets _moving to false so update() stops calling run().  SimpleStepper::stop()
     * also zeroes each stepper's internal step counter as a safety measure.
     */
    void stop() {
        _fl.stop();
        _fr.stop();
        _br.stop();
        _bl.stop();
        _moving       = false;
        _justFinished = false;
    }

    // ── State Queries ─────────────────────────────────────────────────────────

    /** @brief True while any motor is still in motion. */
    bool isMoving() const { return _moving; }

    /**
     * @brief True for exactly one update() cycle after movement completes.
     *
     * firmware.ino reads this flag to emit ACK:MOVE:*:FINISH; the flag is
     * then automatically cleared.
     */
    bool justFinished() {
        if (_justFinished) {
            _justFinished = false;
            return true;
        }
        return false;
    }

private:
    // ── Internal Helpers ─────────────────────────────────────────────────────

    /** Apply the same maximum speed to all four motors. */
    void _setAllSpeeds(float speed) {
        _fl.setMaxSpeed(speed);
        _fr.setMaxSpeed(speed);
        _br.setMaxSpeed(speed);
        _bl.setMaxSpeed(speed);
    }

    /**
     * @brief Read both IR sensors and reduce speed on the drifting side.
     *
     * Left sensor triggered  → rover drifted right → slow LEFT  wheels to steer back.
     * Right sensor triggered → rover drifted left  → slow RIGHT wheels to steer back.
     *
     * When neither sensor sees tape, full speed is applied to both sides.
     * Correction is blended back automatically once the sensor clears.
     */
    void _applyIRCorrection() {
        const float correctedSpeed = MOTOR_MAX_SPEED_SPS * IR_CORRECTION_SPEED_RATIO;
        const float fullSpeed      = MOTOR_MAX_SPEED_SPS;

        const float leftSpeed  = _ir.leftDetected()  ? correctedSpeed : fullSpeed;
        const float rightSpeed = _ir.rightDetected() ? correctedSpeed : fullSpeed;

        _fl.setMaxSpeed(leftSpeed);
        _bl.setMaxSpeed(leftSpeed);
        _fr.setMaxSpeed(rightSpeed);
        _br.setMaxSpeed(rightSpeed);
    }

    // ── Member Variables ──────────────────────────────────────────────────────

    // Step/dir/enable pins passed per-motor from config.h.
    SimpleStepper _fl{ MOTOR_FL_STEP_PIN, MOTOR_FL_DIR_PIN, MOTOR_FL_ENABLE_PIN };
    SimpleStepper _fr{ MOTOR_FR_STEP_PIN, MOTOR_FR_DIR_PIN, MOTOR_FR_ENABLE_PIN };
    SimpleStepper _br{ MOTOR_BR_STEP_PIN, MOTOR_BR_DIR_PIN, MOTOR_BR_ENABLE_PIN };
    SimpleStepper _bl{ MOTOR_BL_STEP_PIN, MOTOR_BL_DIR_PIN, MOTOR_BL_ENABLE_PIN };

    IRSensorPair _ir;   // Sensor reads delegated to sensors.h layer.

    bool _moving       = false;
    bool _justFinished = false;
};

// ═════════════════════════════════════════════════════════════════════════════
// ServoController
// ═════════════════════════════════════════════════════════════════════════════

class ServoController {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Attach the servo and drive it to the home position.
     * Call once from setup().
     */
    void begin() {
        _servo.attach(SERVO_PIN);
        setPosition(SERVO_HOME_POSITION);
        Serial.println(F("SERVO:READY"));
    }

    // ── Command Interface ─────────────────────────────────────────────────────

    /**
     * @brief Move the servo to a position in the 0–100 API range.
     *
     * Linearly maps to [SERVO_ANGLE_MIN, SERVO_ANGLE_MAX].  Adjust those
     * constants in config.h after physical calibration.
     *
     * @param position  0 = full left of frame, 100 = full right of frame.
     */
    void setPosition(uint8_t position) {
        position      = constrain(position, 0U, 100U);
        _lastPosition = position;

        const uint8_t angle = static_cast<uint8_t>(
            map(position, 0, 100, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
        );
        _servo.write(angle);
    }

    /** @brief Last commanded position (0–100). */
    uint8_t getPosition() const { return _lastPosition; }

private:
    Servo   _servo;
    uint8_t _lastPosition = SERVO_HOME_POSITION;
};

// ═════════════════════════════════════════════════════════════════════════════
// PumpController
// ═════════════════════════════════════════════════════════════════════════════

class PumpController {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Configure L298N output pins and ensure the pump is off at boot.
     * Call once from setup().
     */
    void begin() {
        pinMode(PUMP_IN1_PIN, OUTPUT);
        pinMode(PUMP_IN2_PIN, OUTPUT);
        _halt();
        Serial.println(F("PUMP:READY"));
    }

    // ── Non-blocking Update ───────────────────────────────────────────────────

    /**
     * @brief Must be called every iteration of loop().
     *
     * Checks whether the timed pump run has expired and shuts down if so.
     * Returns immediately; never blocks.
     */
    void update() {
        if (!_running) return;

        if (millis() - _startMs >= _durationMs) {
            _halt();
            _running      = false;
            _justFinished = true;
        }
    }

    // ── Command Interface ─────────────────────────────────────────────────────

    /**
     * @brief Start the pump in the specified direction for a set duration.
     *
     * If the pump is already running, the previous run is cancelled and the
     * new command takes effect immediately — this prevents command-queuing
     * ambiguity.
     *
     * @param direction  "FORWARD" (dispense) or "BACKWARD" (retract).
     * @param seconds    Run duration, clamped to PUMP_MAX_DURATION_S.
     */
    void run(const char* direction, uint8_t seconds) {
        _halt();  // Clean state before starting.

        seconds = constrain(seconds, 0U, PUMP_MAX_DURATION_S);
        if (seconds == 0U) return;

        if (strcmp(direction, "FORWARD") == 0) {
            digitalWrite(PUMP_IN1_PIN, HIGH);
            digitalWrite(PUMP_IN2_PIN, LOW);
        } else {
            // BACKWARD — retract / suck back.
            digitalWrite(PUMP_IN1_PIN, LOW);
            digitalWrite(PUMP_IN2_PIN, HIGH);
        }

        _startMs      = millis();
        _durationMs   = static_cast<uint32_t>(seconds) * 1000UL;
        _running      = true;
        _justFinished = false;
    }

    /** @brief Immediately cut pump power regardless of remaining duration. */
    void stop() {
        _halt();
        _running      = false;
        _justFinished = false;
    }

    // ── State Queries ─────────────────────────────────────────────────────────

    /** @brief True while the pump is actively running. */
    bool isRunning() const { return _running; }

    /**
     * @brief True for exactly one update() cycle after a timed run completes.
     *
     * firmware.ino reads this flag to emit ACK:PUMP:FINISH; the flag is
     * then automatically cleared.
     */
    bool justFinished() {
        if (_justFinished) { _justFinished = false; return true; }
        return false;
    }

private:
    // ── Internal Helpers ─────────────────────────────────────────────────────

    /** Both inputs LOW → L298N output coasts (no active braking). */
    void _halt() {
        digitalWrite(PUMP_IN1_PIN, LOW);
        digitalWrite(PUMP_IN2_PIN, LOW);
    }

    // ── Member Variables ──────────────────────────────────────────────────────

    uint32_t _startMs     = 0UL;
    uint32_t _durationMs  = 0UL;
    bool     _running     = false;
    bool     _justFinished = false;
};
