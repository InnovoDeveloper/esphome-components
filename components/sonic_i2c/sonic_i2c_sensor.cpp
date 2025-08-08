#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RCWL-9620 Sonic I2C sensor (30ms compliant)...");
  
  // Initialize state machine
  this->measurement_state_ = MeasurementState::IDLE;
  this->last_trigger_time_ = 0;
  
  // Test basic I2C communication (quick test, <5ms)
  uint8_t test_data;
  auto result = this->read_register(RCWL9620_REG_DISTANCE_HIGH, &test_data, 1);
  if (result != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to communicate with RCWL-9620 at address 0x%02X (Error: %d)", 
             this->address_, result);
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "RCWL-9620 sensor setup complete - I2C communication OK");
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "RCWL-9620 Sonic I2C Sensor (30ms Compliant):");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
  ESP_LOGCONFIG(TAG, "  Range: 2cm - 450cm");
  ESP_LOGCONFIG(TAG, "  Measurement Time: 65ms (non-blocking)");
  ESP_LOGCONFIG(TAG, "  Max Blocking: <30ms per call");
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with RCWL-9620 failed!");
  }
}

void SonicI2CSensor::update() {
  // Start the measurement state machine
  this->measurement_state_ = MeasurementState::TRIGGER;
  this->process_measurement_();
}

void SonicI2CSensor::loop() {
  // Continue measurement process without blocking
  if (this->measurement_state_ != MeasurementState::IDLE) {
    this->process_measurement_();
  }
}

void SonicI2CSensor::process_measurement_() {
  uint32_t now = millis();
  
  switch (this->measurement_state_) {
    case MeasurementState::IDLE:
      // Nothing to do
      break;
      
    case MeasurementState::TRIGGER:
      ESP_LOGV(TAG, "Triggering measurement...");
      
      // Quick trigger command (<5ms)
      auto err = this->write_register(RCWL9620_CMD_TRIGGER, nullptr, 0);
      if (err != i2c::ERROR_OK) {
        ESP_LOGW(TAG, "Trigger command failed: %d", err);
        this->measurement_state_ = MeasurementState::IDLE;
        this->publish_state(NAN);
        return;
      }
      
      this->last_trigger_time_ = now;
      this->measurement_state_ = MeasurementState::WAITING;
      ESP_LOGV(TAG, "Trigger sent, waiting for measurement...");
      break;
      
    case MeasurementState::WAITING:
      // Wait for measurement to complete (65ms total)
      if (now - this->last_trigger_time_ >= MEASUREMENT_DELAY_MS) {
        this->measurement_state_ = MeasurementState::READING;
        ESP_LOGV(TAG, "Measurement time elapsed, attempting read...");
      }
      // Don't block here - just return and check again on next loop
      break;
      
    case MeasurementState::READING:
      // Attempt to read data (must complete in <30ms)
      float distance_cm;
      if (this->read_distance_quick_(distance_cm)) {
        // Convert from centimeters to meters
        float distance_meters = distance_cm / 100.0f;
        
        // Validate reading (RCWL-9620 range: 2cm to 450cm)
        if (distance_cm >= 2.0f && distance_cm <= 450.0f) {
          this->publish_state(distance_meters);
          ESP_LOGD(TAG, "Distance: %.3f m (%.1f cm)", distance_meters, distance_cm);
        } else {
          ESP_LOGW(TAG, "Distance reading out of range: %.1f cm", distance_cm);
          this->publish_state(NAN);
        }
      } else {
        ESP_LOGW(TAG, "Failed to read distance data from RCWL-9620");
        this->publish_state(NAN);
      }
      
      this->measurement_state_ = MeasurementState::IDLE;
      break;
  }
}

bool SonicI2CSensor::read_distance_quick_(float &distance_cm) {
  // All reads must complete in <30ms total
  uint8_t data[2];
  i2c::ErrorCode err;
  uint32_t start_time = millis();
  
  // Method 1: Two-byte read (typically <10ms)
  err = this->read_register(RCWL9620_REG_DISTANCE_HIGH, data, 2);
  if (err == i2c::ERROR_OK) {
    uint16_t raw_distance = (data[0] << 8) | data[1];
    distance_cm = static_cast<float>(raw_distance);
    
    ESP_LOGV(TAG, "Quick 2-byte read: 0x%02X%02X, Distance=%.1fcm (took %dms)", 
             data[0], data[1], distance_cm, millis() - start_time);
    
    if (distance_cm > 0 && distance_cm <= 500) {
      return true;
    }
  }
  
  // Method 2: Separate byte reads (if we have time left)
  if (millis() - start_time < 20) {  // Leave 10ms buffer
    uint8_t high_byte, low_byte;
    err = this->read_register(RCWL9620_REG_DISTANCE_HIGH, &high_byte, 1);
    if (err == i2c::ERROR_OK) {
      err = this->read_register(RCWL9620_REG_DISTANCE_LOW, &low_byte, 1);
      if (err == i2c::ERROR_OK) {
        uint16_t raw_distance = (high_byte << 8) | low_byte;
        distance_cm = static_cast<floa