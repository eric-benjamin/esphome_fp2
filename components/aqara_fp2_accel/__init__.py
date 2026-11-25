import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, binary_sensor, number, button
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
    UNIT_DEGREES,
    DEVICE_CLASS_VIBRATION,
    DEVICE_CLASS_TAMPER,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "number", "button"]

CONF_ILLUMINANCE = "illuminance"

CONF_UPDATE_INTERVAL = "update_interval"

aqara_fp2_accel_ns = cg.esphome_ns.namespace("aqara_fp2_accel")
AqaraFP2Accel = aqara_fp2_accel_ns.class_(
    "AqaraFP2Accel", cg.Component
)
FP2AccelNumber = aqara_fp2_accel_ns.class_("FP2AccelNumber", number.Number, cg.Component)
FP2AccelBaselineButton = aqara_fp2_accel_ns.class_("FP2AccelBaselineButton", button.Button, cg.Component)
# key: (kind, default, min, max, step, unit, icon)
TAMPER_NUMBERS = {
    "tamper_threshold": (0, 3.0, 0.5, 45.0, 0.5, "°", "mdi:angle-acute"),
    "tamper_hold": (1, 3.0, 1.0, 60.0, 1.0, "s", "mdi:timer-outline"),
}
CONF_INITIAL_VALUE = "initial_value"

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
        # The stock firmware's vibration flag (knock). Measured 2026-11-25: fires on
        # corrupted reads, 10 x in 90 min on an untouched unit; not a tamper signal.
        cv.Optional("vibration"): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_VIBRATION
        ),
        # Tamper: gravity vector vs a baseline stored in flash (see the header)
        cv.Optional("tamper"): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_TAMPER
        ),
        cv.Optional("tamper_deviation"): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=1,
            icon="mdi:rotate-3d-variant",
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("tamper_baseline_button"): button.button_schema(
            FP2AccelBaselineButton, icon="mdi:crosshairs-gps", entity_category=ENTITY_CATEGORY_CONFIG
        ),
        **{
            cv.Optional(k): number.number_schema(
                FP2AccelNumber, unit_of_measurement=unit, icon=icon, entity_category=ENTITY_CATEGORY_CONFIG
            ).extend({cv.Optional(CONF_INITIAL_VALUE, default=default): cv.float_range(min=lo, max=hi)})
            for k, (_, default, lo, hi, _, unit, icon) in TAMPER_NUMBERS.items()
        },
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
    if "tamper" in config:
        bs = await binary_sensor.new_binary_sensor(config["tamper"])
        cg.add(var.set_tamper_sensor(bs))
    if "tamper_deviation" in config:
        sens = await sensor.new_sensor(config["tamper_deviation"])
        cg.add(var.set_tamper_deviation_sensor(sens))
    if "tamper_baseline_button" in config:
        btn = await button.new_button(config["tamper_baseline_button"])
        await cg.register_component(btn, config["tamper_baseline_button"])
        cg.add(btn.set_parent(var))
    for key, (kind, default, lo, hi, step, _, _) in TAMPER_NUMBERS.items():
        if key in config:
            num = await number.new_number(config[key], min_value=lo, max_value=hi, step=step)
            await cg.register_component(num, config[key])
            cg.add(num.set_parent(var, kind, config[key][CONF_INITIAL_VALUE]))
