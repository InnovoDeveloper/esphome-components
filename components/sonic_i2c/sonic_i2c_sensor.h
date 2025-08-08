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
  // High-speed measurement state machine
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
  bool read_distance_fast_(float &distance);  // Optimized for speed
  void enable_continuous_mode_();  // Try to enable faster sampling
  
  // RCWL-9620 constants optimized for speed
  static const uint8_t RCWL9620_REG_DISTANCE_HIGH = 0x00;
  static const uint8_t RCWL9620_REG_DISTANCE_LOW = 0x01;
  static const uint8_t RCWL9620_CMD_TRIGGER = 0x01;
  
  // High-speed timing - reduced from 65ms to minimum viable
  static const uint32_t MIN_MEASUREMENT_DELAY_MS = 25;  // Minimum for reliable reading
};

}  // namespace sonic_i2c
}  // namespace esphome