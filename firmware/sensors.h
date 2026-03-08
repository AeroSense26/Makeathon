/**
 * @file    sensors.h
 * @brief   SEN0546 temperature / humidity sensor driver with rolling average.
 *
 * The SensorManager class samples the DFRobot SEN0546 (SHT40-based I²C sensor)
 * once per second in a non-blocking fashion and maintains a ring-buffer of the
 * last SENSOR_SAMPLE_COUNT readings.  Callers invoke getAverages() at any time
 * to retrieve the pre-computed rolling mean — no blocking read is ever needed
 * at query time.
 *
 * ── Required library ─────────────────────────────────────────────────────────
 *   Install "DFRobot_EnvironmentalSensor" via the Arduino Library Manager, OR
 *   "Adafruit SHT4x" if using the Adafruit-branded module.
 *
 *   This file assumes the Adafruit_SHT4x API.  If you use the DFRobot library,
 *   replace the three sensor calls in _takeSample() accordingly:
 *     DFRobot_EnvironmentalSensor sensor(/*i2c addr*\/);
 *     sensor.getTemperature(TEMP_C)
 *     sensor.getHumidity()
 * ─────────────────────────────────────────────────────────────────────────────
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT4x.h>   // ← install "Adafruit SHT4x" from Library Manager

#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────

class SensorManager {
public:
    // ── Lifecycle ────────────────────────────────────────────────────────────

    /**
     * @brief Initialise the I²C bus and sensor hardware.
     *
     * Call once from setup().  Halts with a Serial error message if the sensor
     * cannot be found on the bus — this prevents the rover from operating
     * without valid environmental data.
     */
    void begin() {
        Wire.begin();
        if (!_sht4.begin()) {
            Serial.println(F("ERR:SENSOR:SHT4X_NOT_FOUND"));
            // Stall here so the operator can see the error and investigate.
            while (true) { delay(1000); }
        }
        // High-precision mode, no heater — appropriate for ambient sensing.
        _sht4.setPrecision(SHT4X_HIGH_PRECISION);
        _sht4.setHeater(SHT4X_NO_HEATER);

        // Pre-populate the ring buffer with a real reading so that the first
        // call to getAverages() returns a meaningful value immediately.
        _primeBuffer();

        Serial.println(F("SENSOR:READY"));
    }

    // ── Non-blocking Update ───────────────────────────────────────────────────

    /**
     * @brief Must be called every iteration of loop().
     *
     * Checks whether SENSOR_SAMPLE_INTERVAL_MS has elapsed; if so, takes a
     * new reading and updates the rolling average accumulators.  This function
     * returns immediately without blocking.
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
     *
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

    /** Blocks briefly to fill the ring buffer before entering the main loop. */
    void _primeBuffer() {
        for (uint8_t i = 0; i < SENSOR_SAMPLE_COUNT; ++i) {
            _takeSample();
            delay(SENSOR_SAMPLE_INTERVAL_MS);
        }
    }

    /** Perform one sensor read and slot it into the ring buffer. */
    void _takeSample() {
        sensors_event_t humEvent, tempEvent;
        if (!_sht4.getEvent(&humEvent, &tempEvent)) {
            // Sensor communication failure — skip this slot rather than
            // poisoning the buffer with garbage data.
            Serial.println(F("WARN:SENSOR:READ_FAILED"));
            return;
        }

        // Advance the ring-buffer head (wraps around).
        _head = (_head + 1U) % SENSOR_SAMPLE_COUNT;

        // Subtract the slot's old value from the running sums before overwrite.
        _sumTemp  -= _tempBuf[_head];
        _sumHumid -= _humidBuf[_head];

        // Write new sample.
        _tempBuf[_head]  = tempEvent.temperature;
        _humidBuf[_head] = humEvent.relative_humidity;

        // Add new values to running sums.
        _sumTemp  += _tempBuf[_head];
        _sumHumid += _humidBuf[_head];

        // Track how many valid slots we have (reaches SENSOR_SAMPLE_COUNT
        // after the first full revolution and stays there).
        if (_filledSlots < SENSOR_SAMPLE_COUNT) {
            ++_filledSlots;
        }
    }

    // ── Member Variables ──────────────────────────────────────────────────────

    Adafruit_SHT4x _sht4;

    float   _tempBuf [SENSOR_SAMPLE_COUNT] = {};
    float   _humidBuf[SENSOR_SAMPLE_COUNT] = {};

    float   _sumTemp    = 0.0f;
    float   _sumHumid   = 0.0f;

    uint8_t  _head       = 0U;                  // Points to most-recent slot.
    uint8_t  _filledSlots = 0U;                 // Guards against div-by-zero on startup.

    uint32_t _lastSampleMs = 0U;
};
