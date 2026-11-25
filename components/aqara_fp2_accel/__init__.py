import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
    UNIT_DEGREES,
    DEVICE_CLASS_VIBRATION,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

CONF_ILLUMINANCE = "illuminance"

CONF_UPDATE_INTERVAL = "update_interval"

aqara_fp2_accel_ns = cg.esphome_ns.namespace("aqara_fp2_accel")
AqaraFP2Accel = aqara_fp2_accel_ns.class_(
    "AqaraFP2Accel", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AqaraFP2Accel),
        cv.Optional(CONF_UPDATE_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ILLUMINANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Orientation class the accelerometer resolves (UP, SIDE, DOWN_TILT, ...):
        # this is what the radar is told about how the housing sits.
        cv.Optional("orientation"): text_sensor.text_sensor_schema(
            icon="mdi:axis-arrow", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        # Tilt angle in degrees (0 = housing horizontal, 90 = upright on a wall)
        cv.Optional("tilt_angle"): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=0,
            icon="mdi:angle-acute",
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # The stock firmware's vibration flag (knock / tamper)
        cv.Optional("vibration"): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_VIBRATION
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # ESPHome >= 2026.9 prunes unused ESP-IDF components; we use the
    # i2c_master driver directly, so pull it back in.
    try:
        from esphome.components.esp32 import include_builtin_idf_component
        include_builtin_idf_component("esp_driver_i2c")
    except ImportError:
        pass

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set update interval in milliseconds
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL].total_milliseconds))

    if CONF_ILLUMINANCE in config:
        sens = await sensor.new_sensor(config[CONF_ILLUMINANCE])
        cg.add(var.set_illuminance_sensor(sens))
    if "orientation" in config:
        ts = await text_sensor.new_text_sensor(config["orientation"])
        cg.add(var.set_orientation_sensor(ts))
    if "tilt_angle" in config:
        sens = await sensor.new_sensor(config["tilt_angle"])
        cg.add(var.set_tilt_angle_sensor(sens))
    if "vibration" in config:
        bs = await binary_sensor.new_binary_sensor(config["vibration"])
        cg.add(var.set_vibration_sensor(bs))
