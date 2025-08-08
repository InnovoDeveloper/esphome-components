#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"

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
  this->last_measurement_time_ = millis();
}

void SonicI2CSensor::loop() {
  // Non-blocking state machine to handle the measurement timing
  uint32_t now = millis();
  
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
        // Read the result (fast operation, <10ms)
        uint8_t data[2];
        bool success = false;
        
        // Method 1: Standard read from register 0x00
        if (this->read_bytes(0x00, data, 2)) {
          uint16_t distance_mm = (data[0] << 8) | data[1];
          float distance_m = distance_mm / 1000.0f;
          
          // RCWL-9620 valid range: 20mm to 4500mm
          if (distance_mm >= 20 && distance_mm <= 4500) {
            this->publish_state(distance_m);
            ESP_LOGD(TAG, "Distance: %d mm (%.3f m)", distance_mm, distance_m);
            success = true;
          } else {
            ESP_LOGV(TAG, "Out of range: %d mm", distance_mm);
          }
        }
        
        // Method 2: Try different interpretation if first failed
        if (!success && (data[0] != 0 || data[1] != 0)) {
          // Maybe it's in cm, not mm
          uint16_t distance_cm = (data[0] << 8) | data[1];
          float distance_m = distance_cm / 100.0f;
          
          if (distance_cm >= 2 && distance_cm <= 450) {
            this->publish_state(distance_m);
            ESP_LOGD(TAG, "Distance: %d cm (%.3f m)", distance_cm, distance_m);
            success = true;
          }
        }
        
        if (!success) {
          ESP_LOGW(TAG, "Failed to read valid distance (got: 0x%02X%02X)", data[0], data[1]);
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