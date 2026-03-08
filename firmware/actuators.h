/**
 * @file    actuators.h
 * @brief   Drivetrain, servo nozzle, and pump controllers for the rover.
 *
 * Three independent controller classes are provided:
 *
 *   DrivetrainController
 *     Manages 4× AccelStepper instances (tank-drive layout) and continuously
 *     reads the 2× TCRT5000 IR sensors to apply speed correction in real time.
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
 * ── Required library ─────────────────────────────────────────────────────────
 *   Install "AccelStepper" by Mike McCauley via the Arduino Library Manager.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

#include "config.h"

// ═════════════════════════════════════════════════════════════════════════════
// DrivetrainController
// ═════════════════════════════════════════════════════════════════════════════

class DrivetrainController {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Initialise all four steppers and IR sensor pins.
     * Call once from setup().
     */
    void begin() {
        // Configure steppers (AccelStepper::DRIVER = external step/dir driver).
        _fl.setMaxSpeed(MOTOR_MAX_SPEED_SPS);
        _fl.setAcceleration(MOTOR_ACCELERATION_SPSS);
        _fl.setPinsInverted(MOTOR_FL_INVERT_DIR, false, true);  // last arg: enable inverted (RAMPS EN is active-LOW)
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

        // IR sensors — INPUT_PULLUP provides noise immunity when tape is absent.
        pinMode(IR_LEFT_PIN,  INPUT_PULLUP);
        pinMode(IR_RIGHT_PIN, INPUT_PULLUP);

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

        // run() steps the motor if it is due for a step; returns true while
        // still in motion.
        const bool flRunning = _fl.run();
        const bool frRunning = _fr.run();
        const bool brRunning = _br.run();
        const bool blRunning = _bl.run();

        if (!flRunning && !frRunning && !brRunning && !blRunning) {
            _moving      = false;
            _justFinished = true;
            // Restore nominal speeds for the next command.
            _setAllSpeeds(MOTOR_MAX_SPEED_SPS);
        }
    }

    // ── Command Interface ─────────────────────────────────────────────────────

    /**
     * @brief Queue a relative movement command.
     *
     * @param direction  "FORWARD" or "BACKWARD" (case-sensitive).
     * @param steps      Number of steps to travel (unsigned; direction sets sign).
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
     * @brief Immediately decelerate all motors to a stop.
     *
     * AccelStepper's stop() inserts a deceleration ramp; the motors do not
     * halt abruptly which protects the drivetrain and avoids step loss.
     */
    void stop() {
        _fl.stop();
        _fr.stop();
        _br.stop();
        _bl.stop();
        _moving       = false;
        _justFinished = false;
    }

    // ── State Queries ────────────────────────────────────────────────────────

    /** @brief True while any motor is still in motion. */
    bool isMoving() const { return _moving; }

    /**
     * @brief True for exactly one update() cycle after movement completes.
     *
     * The firmware.ino main loop reads this flag to emit ACK:MOVE:*:FINISH,
     * then the flag is automatically cleared.
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

    /** Set all four motors to the same maximum speed. */
    void _setAllSpeeds(float speed) {
        _fl.setMaxSpeed(speed);
        _fr.setMaxSpeed(speed);
        _br.setMaxSpeed(speed);
        _bl.setMaxSpeed(speed);
    }

    /**
     * @brief Read both IR sensors and reduce speed on the drifting side.
     *
     * Left sensor triggered  → rover drifted right → slow LEFT wheels.
     * Right sensor triggered → rover drifted left  → slow RIGHT wheels.
     *
     * Correction is applied only when the motor is actively moving; the full
     * speed is restored automatically once the sensor clears.
     */
    void _applyIRCorrection() {
        const bool leftTape  = (digitalRead(IR_LEFT_PIN)  == IR_TAPE_LEVEL);
        const bool rightTape = (digitalRead(IR_RIGHT_PIN) == IR_TAPE_LEVEL);

        const float correctedSpeed = MOTOR_MAX_SPEED_SPS * IR_CORRECTION_SPEED_RATIO;
        const float fullSpeed      = MOTOR_MAX_SPEED_SPS;

        // Left side (FL + BL)
        const float leftSpeed  = leftTape  ? correctedSpeed : fullSpeed;
        // Right side (FR + BR)
        const float rightSpeed = rightTape ? correctedSpeed : fullSpeed;

        _fl.setMaxSpeed(leftSpeed);
        _bl.setMaxSpeed(leftSpeed);
        _fr.setMaxSpeed(rightSpeed);
        _br.setMaxSpeed(rightSpeed);
    }

    // ── Member Variables ──────────────────────────────────────────────────────

    // AccelStepper::DRIVER mode: external step/dir driver (DRV8825).
    AccelStepper _fl{ AccelStepper::DRIVER, MOTOR_FL_STEP_PIN, MOTOR_FL_DIR_PIN };
    AccelStepper _fr{ AccelStepper::DRIVER, MOTOR_FR_STEP_PIN, MOTOR_FR_DIR_PIN };
    AccelStepper _br{ AccelStepper::DRIVER, MOTOR_BR_STEP_PIN, MOTOR_BR_DIR_PIN };
    AccelStepper _bl{ AccelStepper::DRIVER, MOTOR_BL_STEP_PIN, MOTOR_BL_DIR_PIN };

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
     * The value is linearly mapped to [SERVO_ANGLE_MIN, SERVO_ANGLE_MAX].
     * Adjust those constants in config.h after physical calibration.
     *
     * @param position  0 = full left of frame, 100 = full right of frame.
     */
    void setPosition(uint8_t position) {
        position       = constrain(position, 0U, 100U);
        _lastPosition  = position;

        // Map 0–100 → SERVO_ANGLE_MIN–SERVO_ANGLE_MAX
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
     * new command takes effect immediately — this prevents command queuing
     * ambiguity.
     *
     * @param direction  "FORWARD" (dispense) or "BACKWARD" (retract).
     * @param seconds    Run duration, clamped to PUMP_MAX_DURATION_S.
     */
    void run(const char* direction, uint8_t seconds) {
        _halt();  // Ensure clean state before starting.

        seconds = constrain(seconds, 0U, PUMP_MAX_DURATION_S);
        if (seconds == 0U) return;

        if (strcmp(direction, "FORWARD") == 0) {
            digitalWrite(PUMP_IN1_PIN, HIGH);
            digitalWrite(PUMP_IN2_PIN, LOW);
        } else {
            // BACKWARD — retract / suck back
            digitalWrite(PUMP_IN1_PIN, LOW);
            digitalWrite(PUMP_IN2_PIN, HIGH);
        }

        _startMs     = millis();
        _durationMs  = static_cast<uint32_t>(seconds) * 1000UL;
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
     * The firmware.ino main loop reads this flag to emit ACK:PUMP:FINISH,
     * then the flag is automatically cleared.
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

    /** Cut all drive signals — both inputs LOW coasts the L298N output. */
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
