#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Sonic I2C Sensor...");
  
  // Simple setup - just test basic I2C communication
  // Don't fail setup, let it try during operation
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Sonic I2C Sensor:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
}

void SonicI2CSensor::update() {
  uint16_t raw_distance;
  
  // Method 1: Try standard ESPHome I2C read
  if (this->read_byte_16(0x00, &raw_distance)) {
    float distance = raw_distance / 10.0f;  // Convert to cm
    distance = distance / 100.0f;  // Convert to meters
    
    if (distance >= 0.02f && distance <= 4.5f) {  // 2cm to 450cm range
      this->publish_state(distance);
      ESP_LOGD(TAG, "Distance: %.3f m", distance);
      return;
    }
  }
  
  // Method 2: Try reading as separate bytes
  uint8_t data[2];
  if (this->read_bytes(0x00, data, 2)) {
    raw_distance = (data[0] << 8) | data[1];
    float distance = raw_distance / 10.0f;  // Convert to cm
    distance = distance / 100.0f;  // Convert to meters
    
    if (distance >= 0.02f && distance <= 4.5f) {
      this->publish_state(distance);
      ESP_LOGD(TAG, "Distance: %.3f m", distance);
      return;
    }
  }
  
  // Method 3: Try without conversion (direct cm reading)
  if (this->read_bytes(0x00, data, 2)) {
    raw_distance = (data[0] << 8) | data[1];
    float distance = static_cast<float>(raw_distance) / 100.0f;  // Direct to meters
    
    if (distance >= 0.02f && distance <= 4.5f) {
      this->publish_state(distance);
      ESP_LOGD(TAG, "Distance: %.3f m", distance);
      return;
    }
  }
  
  ESP_LOGW(TAG, "Failed to read valid distance");
  this->publish_state(NAN);
}

float SonicI2CSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

}  // namespace sonic_i2c
}  // namespace esphome