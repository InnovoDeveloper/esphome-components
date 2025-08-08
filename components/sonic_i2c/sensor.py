import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)

DEPENDENCIES = ["i2c"]

# Create the C++ namespace and class
sonic_i2c_ns = cg.esphome_ns.namespace("sonic_i2c")
SonicI2CSensor = sonic_i2c_ns.class_(
    "SonicI2CSensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

# Configuration schema - simplified
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        SonicI2CSensor,
        unit_of_measurement=UNIT_METER,
        accuracy_decimals=3,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(SonicI2CSensor),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0x57))  # Default I2C address
)

async def to_code(config):
    """Generate the C++ code for the component"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    await i2c.register_i2c_device(var, config)