#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include <esp_timer.h> // For esp_timer_get_time()

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

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
  ESP_LOGCONFIG(TAG, "  Range: 2cm - 450cm");
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
      
    case READ_RESULT:
      {
        uint8_t data[2];
        bool success = false;
        
        // Always log and publish the raw value for diagnostics
        if (this->read_bytes(0x00, data, 2)) {
          uint16_t raw = (data[0] << 8) | data[1];
          ESP_LOGW(TAG, "Diagnostic: Raw sensor reading: 0x%04X (%d)", raw, raw);

          // Attempt to interpret as mm
          float distance_m = raw / 1000.0f;
          ESP_LOGD(TAG, "Diagnostic: Interpreted as meters: %.3f m", distance_m);

          // Attempt to interpret as cm
          float distance_m_cm = raw / 100.0f;
          ESP_LOGD(TAG, "Diagnostic: Interpreted as meters (cm): %.3f m", distance_m_cm);

          // Publish raw value as-is (meters, mm interpretation)
          this->publish_state(distance_m);

          // For diagnostic: you may also want to publish both interpretations to separate sensors
          // (if you have multiple output sensors; otherwise, log is sufficient)

          success = true;
        }
        if (!success) {
          ESP_LOGW(TAG, "Failed to read distance (got: 0x%02X%02X)", data[0], data[1]);
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