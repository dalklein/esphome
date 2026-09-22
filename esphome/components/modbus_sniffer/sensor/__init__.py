import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_ID

from .. import ModbusSniffer, modbus_sniffer_ns

DEPENDENCIES = ["modbus_sniffer"]

CONF_MODBUS_SNIFFER_ID = "modbus_sniffer_id"
CONF_REGISTER = "register"
CONF_VALUE_TYPE = "value_type"

ModbusSnifferSensor = modbus_sniffer_ns.class_(
    "ModbusSnifferSensor", sensor.Sensor, cg.Component
)

# Named as modbus_controller names them, so a config reads the same way on either component.
VALUE_TYPES = {"U_WORD": False, "S_WORD": True}

CONFIG_SCHEMA = sensor.sensor_schema(ModbusSnifferSensor).extend(
    {
        cv.GenerateID(CONF_MODBUS_SNIFFER_ID): cv.use_id(ModbusSniffer),
        cv.Required(CONF_ADDRESS): cv.hex_uint8_t,
        cv.Required(CONF_REGISTER): cv.uint16_t,
        cv.Optional(CONF_VALUE_TYPE, default="U_WORD"): cv.enum(
            VALUE_TYPES, upper=True
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MODBUS_SNIFFER_ID])
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_ADDRESS],
        config[CONF_REGISTER],
        VALUE_TYPES[config[CONF_VALUE_TYPE]],
    )
    await sensor.register_sensor(var, config)
    cg.add(parent.add_item(var))
