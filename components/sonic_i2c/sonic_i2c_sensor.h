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
  void loop() override;  // Handle non-blocking measurement timing
  float get_setup_priority() const override;

 protected:
  enum MeasurementState {
    IDLE,
    TRIGGER_MEASUREMENT,
    WAITING_FOR_MEASUREMENT,
    READ_RESULT
  };
  
  MeasurementState measurement_state_;
  uint32_t last_measurement_time_;
};

}  // namespace sonic_i2c
}  // namespace esphome