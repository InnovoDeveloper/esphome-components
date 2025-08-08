#include "sonic_i2c_sensor.h"
#include "esphome/core/log.h"
#include <esp_timer.h>

namespace esphome {
namespace sonic_i2c {

static const char *const TAG = "sonic_i2c";

// Configurable min/max valid range (in mm)
constexpr float MIN_DISTANCE_MM = 20.0f;
constexpr float MAX_DISTANCE_MM = 4500.0f;  // Based on your original file

// Set non-blocking wait time (less than 30ms, as per request)
constexpr uint32_t MEASUREMENT_DELAY_MS = 25;  // 25ms non-blocking wait

void SonicI2CSensor::setup() {
  ESP_LOGCONFIG(TAG, "Ultrasonic Sensor Setup begin");
  this->measurement_state_ = IDLE;
  this->last_measurement_time_ = 0;
}

void SonicI2CSensor::dump_config() {
  LOG_SENSOR(TAG, "Ultrasonic Sensor", this);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "Range: %.2fmm - %.2fmm (%.2fcm - %.2fcm, %.2fin - %.2fin)", MIN_DISTANCE_MM, MAX_DISTANCE_MM, MIN_DISTANCE_MM/10.0f, MAX_DISTANCE_MM/10.0f, MIN_DISTANCE_MM/25.4f, MAX_DISTANCE_MM/25.4f);
  ESP_LOGCONFIG(TAG, "Non-blocking measurement delay: %d ms", MEASUREMENT_DELAY_MS);
}

void SonicI2CSensor::update() {
  // Start measurement cycle
  this->measurement_state_ = TRIGGER_MEASUREMENT;
  this->last_measurement_time_ = esp_timer_get_time() / 1000; // ms
}

void SonicI2CSensor::loop() {
  uint32_t now = esp_timer_get_time() / 1000; // ms

  switch (this->measurement_state_) {
    case IDLE:
      break;

    case TRIGGER_MEASUREMENT: {
      uint8_t val = 0x01;
      if (this->write(&val, 1)) {
        ESP_LOGV(TAG, "Measurement triggered");
      }
      this->measurement_state_ = WAITING_FOR_MEASUREMENT;
      this->last_measurement_time_ = now;
      break;
    }

    case WAITING_FOR_MEASUREMENT: {
      if (now - this->last_measurement_time_ >= MEASUREMENT_DELAY_MS) {  // Wait 25ms
        this->measurement_state_ = READ_RESULT;
      }
      break;
    }

    case READ_RESULT: {
      uint8_t data_buffer[3] = {0, 0, 0};
      bool success = false;
      if (this->read(data_buffer, 3)) {
        uint32_t raw = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];
        float distance_mm = raw / 1000.0f;  // Data seems to be in micrometers
        float distance_cm = distance_mm / 10.0f;
        float distance_m = distance_mm / 1000.0f;
        float distance_in = distance_mm / 25.4f;

        ESP_LOGI(TAG, "Raw: 0x%06X | mm: %.2f | cm: %.2f | m: %.3f | in: %.2f", raw, distance_mm, distance_cm, distance_m, distance_in);

        if (distance_mm >= MIN_DISTANCE_MM && distance_mm <= MAX_DISTANCE_MM) {
          this->publish_state(distance_m); // meters for ESPHome
          ESP_LOGI(TAG, "Published: %.3f m (%.2f cm, %.2f in)", distance_m, distance_cm, distance_in);
          success = true;
        } else {
          ESP_LOGW(TAG, "Incorrect Distance Reading: %.2f mm (%.2f cm, %.2f in)", distance_mm, distance_cm, distance_in);
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