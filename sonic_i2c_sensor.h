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
  float get_setup_priority() const override;

 protected:
  bool read_distance_data_(float &distance);
  static const uint8_t SONIC_I2C_CMD_MEASURE = 0x51;  // Command to trigger measurement
  static const uint8_t SONIC_I2C_REGISTER_DISTANCE = 0x00;  // Register to read distance
  static const uint16_t SONIC_I2C_TIMEOUT_MS = 100;  // Timeout for I2C operations
};

}  // namespace sonic_i2c
}  // namespace esphome