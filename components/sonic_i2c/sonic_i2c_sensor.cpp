#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RCWL-9620 for HIGH-SPEED object tracking...");
  
  // Initialize state machine
  this->measurement_state_ = MeasurementState::IDLE;
  this->last_trigger_time_ = 0;
  this->continuous_mode_ = false;
  
  // Test I2C communication
  uint8_t test_data;
  auto result = this->read_register(RCWL9620_REG_DISTANCE_HIGH, &test_data, 1);
  if (result != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to communicate with RCWL-9620 at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  
  // Try to enable continuous measurement mode (if supported)
  this->enable_continuous_mode_();
  
  ESP_LOGCONFIG(TAG, "High-speed RCWL-9620 setup complete");
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "RCWL-9620 HIGH-SPEED Object Tracking:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
  ESP_LOGCONFIG(TAG, "  Mode: High-Speed Tracking");
  ESP_LOGCONFIG(TAG, "  Min Measurement Time: %dms", MIN_MEASUREMENT_DELAY_MS);
  ESP_LOGCONFIG(TAG, "  Continuous Mode: %s", this->continuous_mode_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Max Update Rate: ~%d Hz", 1000 / MIN_MEASUREMENT_DELAY_MS);
}

void SonicI2CSensor::update() {
  // For high-speed tracking, start measurement immediately
  this->start_measurement_();
}

void SonicI2CSensor::loop() {
  // High-frequency processing for fast object tracking
  this->process_measurement_();
}

void SonicI2CSensor::start_measurement_() {
  if (this->continuous_mode_) {
    // In continuous mode, just read the latest value
    this->measurement_state_ = MeasurementState::READING;
  } else {
    // Trigger new measurement
    this->measurement_state_ = MeasurementState::TRIGGER;
  }
}

void SonicI2CSensor::process_measurement_() {
  uint32_t now = millis();
  
  switch (this->measurement_state_) {
    case MeasurementState::IDLE:
      // For continuous tracking, don't stay idle long
      if (this->continuous_mode_) {
        this->measurement_state_ = MeasurementState::READING;
      }
      break;
      
    case MeasurementState::TRIGGER:
      {
        // Quick trigger for single-shot mode
        auto err = this->write_register(RCWL9620_CMD_TRIGGER, nullptr, 0);
        if (err != i2c::ERROR_OK) {
          ESP_LOGV(TAG, "Trigger failed: %d", err);
          this->measurement_state_ = MeasurementState::IDLE;
          return;
        }
        
        this->last_trigger_time_ = now;
        this->measurement_state_ = MeasurementState::WAITING;
        break;
      }
      
    case MeasurementState::WAITING:
      // Minimum wait time for sensor measurement
      if (now - this->last_trigger_time_ >= MIN_MEASUREMENT_DELAY_MS) {
        this->measurement_state_ = MeasurementState::READING;
      }
      break;
      
    case MeasurementState::READING:
      {
        float distance_cm;
        if (this->read_distance_fast_(distance_cm)) {
          float distance_meters = distance_cm / 100.0f;
          
          // For fast objects, accept wider range to avoid missing detections
          if (distance_cm >= 1.0f && distance_cm <= 500.0f) {
            this->publish_state(distance_meters);
            ESP_LOGV(TAG, "Fast read: %.1fcm (%.3fm)", distance_cm, distance_meters);
          } else {
            // Don't publish NAN for out-of-range in high-speed mode
            // This reduces noise in fast tracking applications
            ESP_LOGV(TAG, "Out of range: %.1fcm", distance_cm);
          }
        }
        
        // Return to appropriate state for next measurement
        if (this->continuous_mode_) {
          // In continuous mode, read again after minimal delay
          delay(2);  // 2ms delay for I2C bus recovery
          this->measurement_state_ = MeasurementState::READING;
        } else {
          this->measurement_state_ = MeasurementState::IDLE;
        }
        break;
      }
  }
}

bool SonicI2CSensor::read_distance_fast_(float &distance_cm) {
  // Optimized for speed - single read method, <15ms
  uint8_t data[2];
  uint32_t start_time = millis();
  
  // Fast single read - most reliable method for RCWL-9620
  auto err = this->read_register(RCWL9620_REG_DISTANCE_HIGH, data, 2);
  if (err == i2c::ERROR_OK) {
    uint16_t raw_distance = (data[0] << 8) | data[1];
    distance_cm = static_cast<float>(raw_distance);
    
    ESP_LOGV(TAG, "Fast read: 0x%02X%02X = %.1fcm (%dms)", 
             data[0], data[1], distance_cm, millis() - start_time);
    
    // Accept any non-zero reading for speed
    if (distance_cm > 0) {
      return true;
    }
  }
  
  // Single fallback attempt if first read failed
  if (millis() - start_time < 10) {
    err = this->read_register(RCWL9620_REG_DISTANCE_HIGH, data, 2);
    if (err == i2c::ERROR_OK) {
      uint16_t raw_distance = (data[0] << 8) | data[1];
      distance_cm = static_cast<float>(raw_distance);
      
      if (distance_cm > 0) {
        return true;
      }
    }
  }
  
  return false;
}

void SonicI2CSensor::enable_continuous_mode_() {
  // Try to enable continuous measurement mode for faster readings
  // This is sensor-specific - may not work on all RCWL-9620 variants
  
  uint8_t continuous_cmd = 0x02;  // Hypothetical continuous mode command
  auto err = this->write_register(continuous_cmd, nullptr, 0);
  if (err == i2c::ERROR_OK) {
    // Test if continuous mode is working by checking for rapid data updates
    delay(50);
    
    uint8_t data1[2], data2[2];
    if (this->read_register(RCWL9620_REG_DISTANCE_HIGH, data1, 2) == i2c::ERROR_OK) {
      delay(20);
      if (this->read_register(RCWL9620_REG_DISTANCE_HIGH, data2, 2) == i2c::ERROR_OK) {
        // If readings are different, continuous mode might be working
        if (data1[0] != data2[0] || data1[1] != data2[1]) {
          this->continuous_mode_ = true;
          ESP_LOGI(TAG, "Continuous mode enabled - faster tracking possible");
          return;
        }
      }
    }
  }
  
  // Continuous mode not available - use standard triggered mode
  this->continuous_mode_ = false;
  ESP_LOGW(TAG, "Continuous mode not available - using triggered mode");
}

float SonicI2CSensor::get_setup_priority() const {
  return setup_priority::DATA;
}

}  // namespace sonic_i2c
}  // namespace esphome