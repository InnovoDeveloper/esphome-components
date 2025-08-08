#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RCWL-9620 Sonic I2C sensor...");
  
  // Initialize state machine
  this->measurement_state_ = MeasurementState::IDLE;
  this->last_trigger_time_ = 0;
  this->continuous_mode_ = false;
  
  // More lenient setup - don't fail if initial communication has issues
  // Many I2C sensors need a few attempts to establish communication
  
  bool communication_ok = false;
  
  // Try multiple communication methods during setup
  for (int attempt = 0; attempt < 3; attempt++) {
    ESP_LOGD(TAG, "Communication test attempt %d/3", attempt + 1);
    
    // Method 1: Try reading distance register
    uint8_t test_data[2];
    if (this->read_bytes_raw(RCWL9620_REG_DISTANCE_HIGH, test_data, 2)) {
      ESP_LOGD(TAG, "Method 1 success: Read bytes 0x%02X, 0x%02X", test_data[0], test_data[1]);
      communication_ok = true;
      break;
    }
    
    // Method 2: Try single byte read
    uint8_t single_byte;
    if (this->read_byte(RCWL9620_REG_DISTANCE_HIGH, &single_byte)) {
      ESP_LOGD(TAG, "Method 2 success: Read single byte 0x%02X", single_byte);
      communication_ok = true;
      break;
    }
    
    // Method 3: Try write then read (some sensors need wake-up)
    if (this->write_byte(RCWL9620_CMD_TRIGGER, 0x00)) {
      delay(30);  // Short delay for sensor response
      if (this->read_bytes_raw(RCWL9620_REG_DISTANCE_HIGH, test_data, 2)) {
        ESP_LOGD(TAG, "Method 3 success: Write+Read got 0x%02X, 0x%02X", test_data[0], test_data[1]);
        communication_ok = true;
        break;
      }
    }
    
    delay(50);  // Wait between attempts
  }
  
  if (!communication_ok) {
    ESP_LOGW(TAG, "Initial I2C communication failed, but continuing setup...");
    ESP_LOGW(TAG, "Sensor may need time to initialize or different communication protocol");
    // Don't mark as failed - let it try during normal operation
  } else {
    ESP_LOGCONFIG(TAG, "I2C communication established successfully");
  }
  
  ESP_LOGCONFIG(TAG, "RCWL-9620 setup complete");
}

void SonicI2CSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "RCWL-9620 Sonic I2C Sensor:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this);
  ESP_LOGCONFIG(TAG, "  Range: 2cm - 450cm");
  ESP_LOGCONFIG(TAG, "  Communication: Robust multi-method");
  ESP_LOGCONFIG(TAG, "  Min Measurement Time: %dms", MIN_MEASUREMENT_DELAY_MS);
}

void SonicI2CSensor::update() {
  // Start measurement process
  this->start_measurement_();
}

void SonicI2CSensor::loop() {
  // Process state machine
  this->process_measurement_();
}

void SonicI2CSensor::start_measurement_() {
  this->measurement_state_ = MeasurementState::TRIGGER;
}

void SonicI2CSensor::process_measurement_() {
  uint32_t now = millis();
  
  switch (this->measurement_state_) {
    case MeasurementState::IDLE:
      // Nothing to do
      break;
      
    case MeasurementState::TRIGGER:
      {
        ESP_LOGV(TAG, "Triggering measurement...");
        
        // Try triggering measurement - don't fail if this doesn't work
        auto err = this->write_byte(RCWL9620_CMD_TRIGGER, 0x00);
        if (err != i2c::ERROR_OK) {
          ESP_LOGV(TAG, "Trigger command failed (%d), trying direct read", err);
        }
        
        this->last_trigger_time_ = now;
        this->measurement_state_ = MeasurementState::WAITING;
        break;
      }
      
    case MeasurementState::WAITING:
      // Wait for measurement completion
      if (now - this->last_trigger_tim