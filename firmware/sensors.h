/**
 * @file    sensors.h
 * @brief   IR line sensors and SEN0546 (SHT40) temperature/humidity driver.
 *
 * No external libraries required — only Wire.h, which is bundled with the
 * Arduino IDE and requires no Library Manager installation.
 *
 * Classes
 * ───────
 *   IRSensorPair   Reads the two TCRT5000 digital line sensors (D16 / D17).
 *                  Owned here because sensors.h is the "all hardware reading"
 *                  layer; correction logic remains in DrivetrainController.
 *
 *   SensorManager  Non-blocking rolling-average driver for the SEN0546
 *                  (SHT40-based) temperature / humidity sensor over raw I²C.
 *
 * @author  Rover Firmware Team
 * @date    2026-03-08
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>   // Bundled with Arduino IDE — no install required.

#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// SHT40 raw I²C driver  (internal implementation detail)
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

static constexpr uint8_t SHT40_I2C_ADDR     = 0x44U;  // SEN0546 default address
static constexpr uint8_t SHT40_CMD_MEAS_HI  = 0xFDU;  // High-precision, no heater
static constexpr uint8_t SHT40_CMD_RESET    = 0x94U;  // Soft reset
static constexpr uint8_t SHT40_MEAS_DELAY_MS = 10U;   // Max measurement time (high prec.)
static constexpr uint8_t SHT40_CRC_POLY     = 0x31U;  // x⁸ + x⁵ + x⁴ + 1
static constexpr uint8_t SHT40_CRC_INIT     = 0xFFU;

/**
 * @brief CRC-8 as specified in the SHT40 datasheet (two-byte input).
 * Polynomial 0x31, initial value 0xFF, no final XOR.
 */
inline uint8_t sht40Crc(uint8_t msb, uint8_t lsb) {
    uint8_t crc = SHT40_CRC_INIT;
    const uint8_t bytes[2] = { msb, lsb };
    for (const uint8_t b : bytes) {
        crc ^= b;
        for (uint8_t i = 0U; i < 8U; ++i) {
            crc = (crc & 0x80U)
                ? static_cast<uint8_t>((crc << 1U) ^ SHT40_CRC_POLY)
                : static_cast<uint8_t>(crc << 1U);
        }
    }
    return crc;
}

struct SHT40Sample {
    float temperatureC;
    float humidityRH;
    bool  valid;
};

/**
 * @brief Trigger one high-precision measurement and return the result.
 *
 * Blocks for ~10 ms while the SHT40 integrates.  This is acceptable because
 * _takeSample() is only called once per SENSOR_SAMPLE_INTERVAL_MS (1 s).
 * Both temperature and humidity CRCs are verified; invalid data is rejected.
 */
inline SHT40Sample sht40Read() {
    SHT40Sample out{ 0.0f, 0.0f, false };

    Wire.beginTransmission(SHT40_I2C_ADDR);
    Wire.write(SHT40_CMD_MEAS_HI);
    if (Wire.endTransmission(true) != 0U) return out;  // NAK or bus error

    delay(SHT40_MEAS_DELAY_MS);

    if (Wire.requestFrom(SHT40_I2C_ADDR, static_cast<uint8_t>(6U)) != 6U) return out;

    const uint8_t t_msb  = static_cast<uint8_t>(Wire.read());
    const uint8_t t_lsb  = static_cast<uint8_t>(Wire.read());
    const uint8_t t_crc  = static_cast<uint8_t>(Wire.read());
    const uint8_t rh_msb = static_cast<uint8_t>(Wire.read());
    const uint8_t rh_lsb = static_cast<uint8_t>(Wire.read());
    const uint8_t rh_crc = static_cast<uint8_t>(Wire.read());

    if (sht40Crc(t_msb, t_lsb) != t_crc || sht40Crc(rh_msb, rh_lsb) != rh_crc) {
        return out;  // CRC mismatch — discard corrupt frame
    }

    const uint16_t raw_t  = static_cast<uint16_t>(
        (static_cast<uint16_t>(t_msb)  << 8U) | t_lsb);
    const uint16_t raw_rh = static_cast<uint16_t>(
        (static_cast<uint16_t>(rh_msb) << 8U) | rh_lsb);

    // Conversion per SHT40 datasheet §4.6
    out.temperatureC = -45.0f + 175.0f * (static_cast<float>(raw_t)  / 65535.0f);
    out.humidityRH   =  -6.0f + 125.0f * (static_cast<float>(raw_rh) / 65535.0f);
    out.humidityRH   = constrain(out.humidityRH, 0.0f, 100.0f);
    out.valid        = true;
    return out;
}

} // namespace detail

// ═════════════════════════════════════════════════════════════════════════════
// IRSensorPair
// ═════════════════════════════════════════════════════════════════════════════

/**
 * @brief Thin wrapper around the two TCRT5000 digital line sensors.
 *
 * Encapsulates pin initialisation and read logic so sensors.h owns all
 * hardware reading.  DrivetrainController holds an instance of this class
 * and consults it each update() tick to apply speed correction — the
 * correction arithmetic itself stays in actuators.h where it belongs.
 */
class IRSensorPair {
public:
    /**
     * @brief Configure both IR sensor pins as inputs with pull-ups.
     * Call once from setup() via DrivetrainController::begin().
     */
    void begin() {
        // INPUT_PULLUP ensures a defined logic level when tape is absent,
        // reducing susceptibility to ambient IR noise.
        pinMode(IR_LEFT_PIN,  INPUT_PULLUP);
        pinMode(IR_RIGHT_PIN, INPUT_PULLUP);
    }

    /** @brief Returns true when the LEFT sensor is over black tape. */
    bool leftDetected()  const {
        return static_cast<uint8_t>(digitalRead(IR_LEFT_PIN))  == IR_TAPE_LEVEL;
    }

    /** @brief Returns true when the RIGHT sensor is over black tape. */
    bool rightDetected() const {
        return static_cast<uint8_t>(digitalRead(IR_RIGHT_PIN)) == IR_TAPE_LEVEL;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
// SensorManager
// ═════════════════════════════════════════════════════════════════════════════

/**
 * @brief Non-blocking rolling-average wrapper for the SEN0546 (SHT40) sensor.
 *
 * Samples the sensor once per SENSOR_SAMPLE_INTERVAL_MS and maintains a
 * ring buffer of the last SENSOR_SAMPLE_COUNT readings.  Callers invoke
 * getAverages() at any time to receive the pre-computed mean — no blocking
 * read is ever needed at query time.
 */
class SensorManager {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Initialise the I²C bus and verify the SHT40 is present.
     *
     * Sends a soft-reset to the SHT40 and checks for an I²C ACK.  If the
     * device does not respond, execution halts with a Serial error message so
     * the operator can investigate before the rover attempts autonomous work.
     */
    void begin() {
        Wire.begin();

        // Probe: issue soft reset and check for ACK.
        Wire.beginTransmission(detail::SHT40_I2C_ADDR);
        Wire.write(detail::SHT40_CMD_RESET);
        if (Wire.endTransmission(true) != 0U) {
            Serial.println(F("ERR:SENSOR:SHT40_NOT_FOUND"));
            while (true) { delay(1000); }  // Halt — operator must investigate.
        }
        delay(2U);  // SHT40 soft-reset time (datasheet: < 1 ms; 2 ms is safe)

        _primeBuffer();
        Serial.println(F("SENSOR:READY"));
    }

    // ── Non-blocking Update ───────────────────────────────────────────────────

    /**
     * @brief Must be called every iteration of loop().
     *
     * Returns immediately between sample intervals.  When the interval elapses,
     * blocks for ~10 ms while the SHT40 performs its measurement.
     */
    void update() {
        const uint32_t now = millis();
        if (now - _lastSampleMs >= SENSOR_SAMPLE_INTERVAL_MS) {
            _lastSampleMs = now;
            _takeSample();
        }
    }

    // ── Data Access ───────────────────────────────────────────────────────────

    /**
     * @brief Retrieve the rolling-average temperature and humidity.
     * @param[out] temperatureC  Average temperature in °C.
     * @param[out] humidityRH    Average relative humidity in %RH.
     */
    void getAverages(float& temperatureC, float& humidityRH) const {
        temperatureC = _sumTemp  / static_cast<float>(_filledSlots);
        humidityRH   = _sumHumid / static_cast<float>(_filledSlots);
    }

    /** @brief Latest single-sample temperature in °C (not averaged). */
    float getLatestTemperature() const { return _tempBuf[_head]; }

    /** @brief Latest single-sample humidity in %RH (not averaged). */
    float getLatestHumidity()    const { return _humidBuf[_head]; }

private:
    // ── Internal Helpers ─────────────────────────────────────────────────────

    /** Blocks to fill the ring buffer before the main loop starts. */
    void _primeBuffer() {
        for (uint8_t i = 0U; i < SENSOR_SAMPLE_COUNT; ++i) {
            _takeSample();
            delay(SENSOR_SAMPLE_INTERVAL_MS);
        }
    }

    /** Perform one SHT40 read and slot the result into the ring buffer. */
    void _takeSample() {
        const detail::SHT40Sample s = detail::sht40Read();
        if (!s.valid) {
            Serial.println(F("WARN:SENSOR:READ_FAILED"));
            return;
        }

        // Advance ring-buffer head (wraps on overflow).
        _head = (_head + 1U) % SENSOR_SAMPLE_COUNT;

        // Subtract the slot's old value from running sums before overwriting.
        _sumTemp  -= _tempBuf[_head];
        _sumHumid -= _humidBuf[_head];

        _tempBuf[_head]  = s.temperatureC;
        _humidBuf[_head] = s.humidityRH;

        _sumTemp  += _tempBuf[_head];
        _sumHumid += _humidBuf[_head];

        if (_filledSlots < SENSOR_SAMPLE_COUNT) { ++_filledSlots; }
    }

    // ── Member Variables ──────────────────────────────────────────────────────

    float    _tempBuf [SENSOR_SAMPLE_COUNT] = {};
    float    _humidBuf[SENSOR_SAMPLE_COUNT] = {};

    float    _sumTemp     = 0.0f;
    float    _sumHumid    = 0.0f;

    uint8_t  _head        = 0U;      // Index of the most-recently written slot.
    uint8_t  _filledSlots = 0U;      // Guards against div-by-zero on startup.

    uint32_t _lastSampleMs = 0UL;
};
