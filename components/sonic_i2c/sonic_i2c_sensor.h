#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace sonic_i2c {

class SonicI2CSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override;

 protected:
  // State machine for measurement
  enum class MeasurementState {
    IDLE,
    TRIGGER,
    WAITING,
    READING
  };
  
  MeasurementState measurement_state_{MeasurementState::IDLE};
  uint32_t last_trigger_time_{0};
  bool continuous_mode_{false};
  
  void start_measurement_();
  void process_measurement_();
  bool read_distance_robust_(float &distance);  // Multiple fallback methods
  
  // Robust I2C helper methods
  bool read_bytes_raw(uint8_t register_addr, uint8_t *data, size_t len);
  bool read_byte(uint8_t register_addr, uint8_t *data);
  bool write_byte(uint8_t register_addr, uint8_t data);
  
  // RCWL-9620 constants
  static const uint8_t RCWL9620_REG_DISTANCE_HIGH = 0x00;
  static const uint8_t RCWL9620_REG_DISTANCE_LOW = 0x01;
  static const uint8_t RCWL9620_CMD_TRIGGER = 0x01;
  static const uint32_t MIN_MEASUREMENT_DELAY_MS = 30;  // Conservative timing
};

}  // namespace sonic_i2c
}  // namespace esphome