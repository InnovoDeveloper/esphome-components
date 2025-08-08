#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Sonic I2C sensor...");
  
  // Test I2C communication
  if (!this->read_byte(0x00).has_value()) {
    ESP_LOGE(TAG, "Failed to communicate with Sonic I2C sensor at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Sonic I2C sensor setup complete");
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Sonic I2C Sensor:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with Sonic I2C sensor failed!");
  }
}

void SonicI2CSensor::update() {
  float distance;
  
  if (this->read_distance_data_(distance)) {
    // Convert from centimeters to meters (assuming sensor returns cm)
    float distance_meters = distance / 100.0f;
    
    // Validate reading (typical ultrasonic range 2cm to 4m)
    if (distance_meters >= 0.02f && distance_meters <= 4.0f) {
      this->publish_state(distance_meters);
      ESP_LOGD(TAG, "Distance: %.3f m (%.1f cm)", distance_meters, distance);
    } else {
      ESP_LOGW(TAG, "Invalid distance reading: %.3f m", distance_meters);
      this->publish_state(NAN);
    }
  } else {
    ESP_LOGW(TAG, "Failed to read distance data");
    this->publish_state(NAN);
  }
}

bool SonicI2CSensor::read_distance_data_(float &distance) {
  // Method 1: Try reading 2 bytes directly (common for ultrasonic sensors)
  uint8_t data[2];
  if (this->read_bytes(SONIC_I2C_REGISTER_DISTANCE, data, 2)) {
    // Combine high and low bytes (big-endian)
    uint16_t raw_distance = (data[0] << 8) | data[1];
    distance = static_cast<float>(raw_distance);
    
    // If distance seems reasonable, return success
    if (distance > 0 && distance < 4000) {  // 0-40m in cm
      return true;
    }
  }
  
  // Method 2: Try triggering measurement first, then reading
  if (this->write_byte(SONIC_I2C_CMD_MEASURE, 0x00)) {
    // Wait for measurement to complete (typical 65ms for ultrasonic)
    delay(70);
    
    if (this->read_bytes(SONIC_I2C_REGISTER_DISTANCE, data, 2)) {
      uint16_t raw_distance = (data[0] << 8) | data[1];
      distance = static_cast<float>(raw_distance);
      
      if (distance > 0 && distance < 4000) {
        return true;
      }
    }
  }
  
  // Method 3: Try alternative register layout (little-endian)
  if (this->read_bytes(SONIC_I2C_REGISTER_DISTANCE, data, 2)) {
    uint16_t raw_distance = (data[1] << 8) | data[0];  // Little-endian
    distance = static_cast<float>(raw_distance);
    
    if (distance > 0 && distance < 4000) {
      return true;
    }
  }
  
  return false;
}

float SonicI2CSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

}  // namespace sonic_i2c
}  // namespace esphome