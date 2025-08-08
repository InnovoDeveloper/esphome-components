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
  void loop() override;  // Non-blocking state machine
  float get_setup_priority() const override;

 protected:
  // State machine for non-blocking measurement
  enum class MeasurementState {
    IDLE,
    TRIGGER,
    WAITING,
    READING
  };
  
  MeasurementState measurement_state_{MeasurementState::IDLE};
  uint32_t last_trigger_time_{0};
  
  void process_measurement_();
  bool read_distance_quick_(float &distance);  // Must complete in <30ms
  
  // RCWL-9620 specific constants
  static const uint8_t RCWL9620_REG_DISTANCE_HIGH = 0x00;
  static const uint8_t RCWL9620_REG_DISTANCE_LOW = 0x01;
  static const uint8_t RCWL9620_CMD_TRIGGER = 0x01;
  static const uint32_t MEASUREMENT_DELAY_MS = 65;  // Sensor measurement time
};

}  // namespace sonic_i2c
}  // namespace esphome