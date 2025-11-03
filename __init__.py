import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_CHANNEL,
    CONF_UPDATE_INTERVAL,
    CONF_TRIGGER_ID,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_WATT,
)
from esphome import automation

DEPENDENCIES = ["wifi", "esp32"]
CODEOWNERS = ["@chinawrj"]

powersync_ns = cg.esphome_ns.namespace("powersync")
PowerSyncComponent = powersync_ns.class_("PowerSyncComponent", cg.Component)

# Trigger for inverter output power adjustment needed event
InverterOutputPowerAdjustmentTrigger = powersync_ns.class_(
    "InverterOutputPowerAdjustmentTrigger", automation.Trigger.template(cg.float_)
)

CONF_BROADCAST_INTERVAL = "broadcast_interval"
CONF_SYSTEM_UPDATE_INTERVAL = "system_update_interval"
CONF_AUTO_ADD_PEER = "auto_add_peer"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_POWER_DECISION_DATA_TIMEOUT = "power_decision_data_timeout"
CONF_POWER_CHANGE_THRESHOLD = "power_change_threshold"
CONF_SOLAR_POWER_THRESHOLD = "solar_power_threshold"
CONF_INVERTER_OUTPUT_POWER_RANGE_MIN = "inverter_output_power_range_min"
CONF_INVERTER_OUTPUT_POWER_RANGE_MAX = "inverter_output_power_range_max"
CONF_ON_INVERTER_OUTPUT_POWER_ADJUSTMENT = "on_inverter_output_power_adjustment"

# TLV sensor configuration
CONF_AC_VOLTAGE_SENSOR = "ac_voltage_sensor"
CONF_AC_CURRENT_SENSOR = "ac_current_sensor"
CONF_AC_FREQUENCY_SENSOR = "ac_frequency_sensor"
CONF_AC_POWER_SENSOR = "ac_power_sensor"
CONF_BUTTON_PRESS_COUNT_SENSOR = "button_press_count_sensor"
CONF_STATE_DURATION_SENSOR = "state_duration_sensor"
CONF_DEVICE_ROLE = "device_role"
CONF_DLT645_RELAY_TRIP_SENSOR = "dlt645_relay_trip_sensor"
CONF_DLT645_RELAY_CLOSE_SENSOR = "dlt645_relay_close_sensor"

# Device Role Enum
DeviceRole = powersync_ns.enum("DeviceRole")
DEVICE_ROLES = {
    "UNKNOWN": DeviceRole.ROLE_UNKNOWN,
    "GRID_INPUT": DeviceRole.ROLE_GRID_INPUT,
    "INVERTER_AC_INPUT": DeviceRole.ROLE_INVERTER_AC_INPUT,
    "INVERTER_AC_OUTPUT": DeviceRole.ROLE_INVERTER_AC_OUTPUT,
    "INVERTER_DC_BATTERY": DeviceRole.ROLE_INVERTER_DC_BATTERY,
    "INVERTER_DC_GRID_POWER": DeviceRole.ROLE_INVERTER_DC_GRID_POWER,
    "SINKER_AC_HEATER": DeviceRole.ROLE_SINKER_AC_HEATER,
    "SINKER_DC_HEATER": DeviceRole.ROLE_SINKER_DC_HEATER,
    "SINKER_AC_VEHICLE_CHARGER": DeviceRole.ROLE_SINKER_AC_VEHICLE_CHARGER,
    "SINKER_DC_VEHICLE_CHARGER": DeviceRole.ROLE_SINKER_DC_VEHICLE_CHARGER,
    "SOLOAR_INVERTER_OUTPUT_TOTAL": DeviceRole.ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PowerSyncComponent),
        cv.Optional(CONF_CHANNEL, default=1): cv.int_range(min=1, max=14),
        cv.Optional(CONF_AUTO_ADD_PEER, default=True): cv.boolean,
        cv.Optional(CONF_BROADCAST_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SYSTEM_UPDATE_INTERVAL, default="100ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_POWER_DECISION_DATA_TIMEOUT, default="60s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_POWER_CHANGE_THRESHOLD, default=100.0): cv.float_range(min=0.0),
        cv.Optional(CONF_SOLAR_POWER_THRESHOLD, default=-10.0): cv.float_,
        cv.Optional(CONF_INVERTER_OUTPUT_POWER_RANGE_MIN, default=-150.0): cv.float_,
        cv.Optional(CONF_INVERTER_OUTPUT_POWER_RANGE_MAX, default=150.0): cv.float_,
        cv.Required(CONF_FIRMWARE_VERSION): cv.string_strict,
        cv.Optional(CONF_AC_VOLTAGE_SENSOR): sensor.sensor_schema(
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_AC_CURRENT_SENSOR): sensor.sensor_schema(
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_AC_FREQUENCY_SENSOR): sensor.sensor_schema(
            device_class=DEVICE_CLASS_FREQUENCY,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_HERTZ,
            accuracy_decimals=2,
        ),
        cv.Optional(CONF_AC_POWER_SENSOR): sensor.sensor_schema(
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_BUTTON_PRESS_COUNT_SENSOR): sensor.sensor_schema(
            icon="mdi:counter",
            accuracy_decimals=0,
            unit_of_measurement="times",
        ),
        cv.Optional(CONF_STATE_DURATION_SENSOR): sensor.sensor_schema(
            icon="mdi:timer-outline",
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=2,
            unit_of_measurement="h",
        ),
        cv.Optional(CONF_DEVICE_ROLE, default="UNKNOWN"): cv.enum(DEVICE_ROLES, upper=True),
        cv.Optional(CONF_DLT645_RELAY_TRIP_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_DLT645_RELAY_CLOSE_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_ON_INVERTER_OUTPUT_POWER_ADJUSTMENT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(InverterOutputPowerAdjustmentTrigger),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set configuration parameters
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_auto_add_peer(config[CONF_AUTO_ADD_PEER]))
    cg.add(var.set_broadcast_interval(config[CONF_BROADCAST_INTERVAL]))
    cg.add(var.set_system_update_interval(config[CONF_SYSTEM_UPDATE_INTERVAL]))
    cg.add(var.set_power_decision_data_timeout(config[CONF_POWER_DECISION_DATA_TIMEOUT]))
    cg.add(var.set_power_change_threshold(config[CONF_POWER_CHANGE_THRESHOLD]))
    cg.add(var.set_solar_power_threshold(config[CONF_SOLAR_POWER_THRESHOLD]))
    cg.add(var.set_inverter_output_power_range_min(config[CONF_INVERTER_OUTPUT_POWER_RANGE_MIN]))
    cg.add(var.set_inverter_output_power_range_max(config[CONF_INVERTER_OUTPUT_POWER_RANGE_MAX]))
    cg.add(var.set_firmware_version(config[CONF_FIRMWARE_VERSION]))
    cg.add(var.set_device_role(config[CONF_DEVICE_ROLE]))

    # Configure optional sensors
    if CONF_AC_VOLTAGE_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_AC_VOLTAGE_SENSOR])
        cg.add(var.set_ac_voltage_sensor(sens))

    if CONF_AC_CURRENT_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_AC_CURRENT_SENSOR])
        cg.add(var.set_ac_current_sensor(sens))

    if CONF_AC_FREQUENCY_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_AC_FREQUENCY_SENSOR])
        cg.add(var.set_ac_frequency_sensor(sens))

    if CONF_AC_POWER_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_AC_POWER_SENSOR])
        cg.add(var.set_ac_power_sensor(sens))

    if CONF_BUTTON_PRESS_COUNT_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_BUTTON_PRESS_COUNT_SENSOR])
        cg.add(var.set_button_press_count_sensor(sens))

    if CONF_STATE_DURATION_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_STATE_DURATION_SENSOR])
        cg.add(var.set_state_duration_sensor(sens))

    # Configure DLT645 relay control binary sensors
    if CONF_DLT645_RELAY_TRIP_SENSOR in config:
        sens = await cg.get_variable(config[CONF_DLT645_RELAY_TRIP_SENSOR])
        cg.add(var.set_dlt645_relay_trip_sensor(sens))

    if CONF_DLT645_RELAY_CLOSE_SENSOR in config:
        sens = await cg.get_variable(config[CONF_DLT645_RELAY_CLOSE_SENSOR])
        cg.add(var.set_dlt645_relay_close_sensor(sens))
    
    # Configure inverter output power adjustment trigger
    for conf in config.get(CONF_ON_INVERTER_OUTPUT_POWER_ADJUSTMENT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.float_, "power_gap_watts")], conf)

    # Add ESP-NOW dependency
    cg.add_library("ESP-NOW", None)