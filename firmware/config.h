/**
 * @file    config.h
 * @brief   Compile-time configuration for the Autonomous Fertilizer Rover firmware.
 *
 * All pin assignments, motion tuning parameters, and protocol constants are
 * centralised here. Adjust values in this file only — never scatter magic
 * numbers throughout the source.
 *
 * Hardware context
 * ────────────────
 *   MCU   : Arduino Mega 2560
 *   Shield : RAMPS 1.4
 *   Drivers: 4× DRV8825 (X / Y / Z / E0 slots)
 */

#pragma once

#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Serial
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t SERIAL_BAUD_RATE    = 115200U;

/** Maximum byte length of one inbound serial command (including null terminator). */
static constexpr uint8_t  SERIAL_BUFFER_SIZE  = 64U;

// ─────────────────────────────────────────────────────────────────────────────
// RAMPS 1.4 – Stepper Motor Pin Assignments
//
// Wheel positions (viewed from above, rover moving "forward" = up):
//
//          [FL]       [FR]
//           E0          X
//
//          [BL]       [BR]
//           Z           Y
//
// Right-side motors are physically mirrored, so their direction must be
// inverted to achieve a consistent "forward" behaviour — see INVERT flags.
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left  (E0 slot)
static constexpr uint8_t MOTOR_FL_STEP_PIN   = 26U;
static constexpr uint8_t MOTOR_FL_DIR_PIN    = 28U;
static constexpr uint8_t MOTOR_FL_ENABLE_PIN = 24U;
static constexpr bool    MOTOR_FL_INVERT_DIR = false;

// Front-Right (X slot)
static constexpr uint8_t MOTOR_FR_STEP_PIN   = 54U;
static constexpr uint8_t MOTOR_FR_DIR_PIN    = 55U;
static constexpr uint8_t MOTOR_FR_ENABLE_PIN = 38U;
static constexpr bool    MOTOR_FR_INVERT_DIR = true;   // mirrored mounting

// Back-Right  (Y slot)
static constexpr uint8_t MOTOR_BR_STEP_PIN   = 60U;
static constexpr uint8_t MOTOR_BR_DIR_PIN    = 61U;
static constexpr uint8_t MOTOR_BR_ENABLE_PIN = 56U;
static constexpr bool    MOTOR_BR_INVERT_DIR = true;   // mirrored mounting

// Back-Left   (Z slot)
static constexpr uint8_t MOTOR_BL_STEP_PIN   = 46U;
static constexpr uint8_t MOTOR_BL_DIR_PIN    = 48U;
static constexpr uint8_t MOTOR_BL_ENABLE_PIN = 62U;
static constexpr bool    MOTOR_BL_INVERT_DIR = false;

// ─────────────────────────────────────────────────────────────────────────────
// Motion Tuning
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Nominal cruising speed in steps per second.
 * DRV8825 at 1/16 micro-step, 200 steps/rev → ~1.5 RPM at 50 sps.
 * Keep this low for a precision fertiliser rover.
 */
static constexpr float MOTOR_MAX_SPEED_SPS      = 200.0f;

/** Acceleration in steps/sec². Gentle ramp to avoid slipping. */
static constexpr float MOTOR_ACCELERATION_SPSS  = 150.0f;

/**
 * Speed multiplier applied to the lagging side during IR correction.
 * Range 0.0–1.0.  Lower = more aggressive correction turn.
 * E.g. 0.45 means the correction side runs at 45 % of max speed.
 */
static constexpr float IR_CORRECTION_SPEED_RATIO = 0.45f;

// ─────────────────────────────────────────────────────────────────────────────
// IR Line Sensors (TCRT5000) — D16 (left) / D17 (right)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t IR_LEFT_PIN  = 16U;
static constexpr uint8_t IR_RIGHT_PIN = 17U;

/**
 * Logic level reported by the TCRT5000 module when black tape is detected.
 * Most modules with onboard comparator output LOW on detection.
 * Flip to HIGH if your module behaves inversely.
 */
static constexpr uint8_t IR_TAPE_LEVEL = LOW;

// ─────────────────────────────────────────────────────────────────────────────
// Servo Nozzle (MG996R) — D11
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t SERVO_PIN             = 11U;

/**
 * Servo pulse-width range in microseconds, mapped to the 0–100 API range.
 * Standard servo: 1 000 µs = full left (position 0), 2 000 µs = full right (position 100).
 * Adjust after physical calibration to match the actual nozzle sweep.
 */
static constexpr uint16_t SERVO_PULSE_MIN_US   = 1000U; // corresponds to position   0
static constexpr uint16_t SERVO_PULSE_MAX_US   = 2000U; // corresponds to position 100

/** Servo rests here at power-on (0 = full left of frame). */
static constexpr uint8_t SERVO_HOME_POSITION   = 0U;

// ─────────────────────────────────────────────────────────────────────────────
// Pump (DC motor via L298N) — IN1: D23 / IN2: D25
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t PUMP_IN1_PIN = 23U;
static constexpr uint8_t PUMP_IN2_PIN = 25U;

/** Safety ceiling: maximum pump run time accepted via command (seconds). */
static constexpr uint8_t PUMP_MAX_DURATION_S   = 60U;

// ─────────────────────────────────────────────────────────────────────────────
// Temperature / Humidity Sensor (SEN0546) — I²C SDA: 20 / SCL: 21
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Rolling-average ring-buffer depth (number of samples retained).
 * Samples are taken every SENSOR_SAMPLE_INTERVAL_MS milliseconds, giving
 * a rolling window of SENSOR_SAMPLE_COUNT × SENSOR_SAMPLE_INTERVAL_MS ms.
 */
static constexpr uint8_t  SENSOR_SAMPLE_COUNT       = 10U;
static constexpr uint32_t SENSOR_SAMPLE_INTERVAL_MS = 1000U;   // 1 s → 10 s window
