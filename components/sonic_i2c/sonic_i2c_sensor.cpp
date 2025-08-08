#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include <esp_timer.h> // For esp_timer_get_time()

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

// Configurable min/max valid range (in mm)
constexpr uint16_t MIN_DISTANCE_MM = 20;
constexpr uint16_t MAX_DISTANCE_MM = 7000;

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Sonic I2C Sensor...");
  this->measurement_state_ = IDLE;
  this->last_measurement_time_ = 0;
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Sonic I2C Sensor:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
  ESP_LOGCONFIG(TAG, "  Range: %dmm - %dmm (%.2fcm - %.2fcm, %.2fin - %.2fin)", MIN_DISTANCE_MM, MAX_DISTANCE_MM, MIN_DISTANCE_MM / 10.0f, MAX_DISTANCE_MM / 10.0f, MIN_DISTANCE_MM / 25.4f, MAX_DISTANCE_MM / 25.4f);
}

void SonicI2CSensor::update() {
  // Start measurement cycle
  this->measurement_state_ = TRIGGER_MEASUREMENT;
  this->last_measurement_time_ = esp_timer_get_time() / 1000; // milliseconds
}

void SonicI2CSensor::loop() {
  // Non-blocking state machine to handle the measurement timing
  uint32_t now = esp_timer_get_time() / 1000; // milliseconds

  switch (this->measurement_state_) {
    case IDLE:
      // Do nothing
      break;

    case TRIGGER_MEASUREMENT:
      // Send trigger command (this is fast, <5ms)
      if (this->write_bytes(0x01, nullptr, 0)) {
        ESP_LOGV(TAG, "Measurement triggered");
      }
      this->measurement_state_ = WAITING_FOR_MEASUREMENT;
      this->last_measurement_time_ = now;
      break;

    case WAITING_FOR_MEASUREMENT:
      // Wait 65ms for measurement to complete (non-blocking)
      if (now - this->last_measurement_time_ >= 65) {
        this->measurement_state_ = READ_RESULT;
      }
      break;

    case READ_RESULT: {
      uint8_t data[2];
      bool success = false;

      if (this->read_bytes(0x00, data, 2)) {
        uint16_t distance_mm = (data[0] << 8) | data[1];
        float distance_cm = distance_mm / 10.0f;
        float distance_m = distance_mm / 1000.0f;
        float distance_in = distance_mm / 25.4f;

        ESP_LOGI(TAG, "Raw: 0x%04X | mm: %u | cm: %.2f | m: %.3f | in: %.2f", distance_mm, distance_mm, distance_cm, distance_m, distance_in);

        if (distance_mm >= MIN_DISTANCE_MM && distance_mm <= MAX_DISTANCE_MM) {
          // Publish in meters (default for ESPHome)
          this->publish_state(distance_m);

          // Additional logging for other units
          ESP_LOGI(TAG, "Published: %.3f m (%.2f cm, %.2f in)", distance_m, distance_cm, distance_in);
          success = true;
        } else {
          ESP_LOGW(TAG, "Out of range: 0x%04X (%u mm, %.2f cm, %.2f in)", distance_mm, distance_mm, distance_cm, distance_in);
          this->publish_state(NAN);
        }
      }

      if (!success) {
        ESP_LOGW(TAG, "Failed to read distance (I2C error or out of range)");
        this->publish_state(NAN);
      }
      this->measurement_state_ = IDLE;
      break;
    }
  }
}

float SonicI2CSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

}  // namespace sonic_i2c
}  // namespace esphome