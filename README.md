# PowerSync Component

A comprehensive ESPHome component that provides ESP-NOW communication with TLV (Type-Length-Value) telemetry broadcasting for ESP32-based PowerSync projects.

## Features

- **ESP-NOW Communication**: Broadcast and receive packets on specified channel (default: channel 1)
- **TLV Data Broadcasting**: Structured telemetry data using 11 different TLV types
- **System Information**: Device ID, firmware version, compile time, MAC address, uptime, memory usage
- **AC Power Measurements**: Voltage, current, frequency, and power with high precision
- **LED Effects**: RGB notifications for packet reception and alerts
- **Configurable Intervals**: Customizable broadcast and system update frequencies

## TLV Data Format

The PowerSync component uses the standardized TLV (Type-Length-Value) format defined in `powersync_tlv_format.h`. This ensures compatibility across different projects and platforms.

## TLV Types Supported

| Type | Description | Data Format |
|------|-------------|-------------|
| 0x01 | System Uptime | uint32 (seconds) |
| 0x03 | Device ID | string |
| 0x04 | Firmware Version | string |
| 0x05 | MAC Address | 6 bytes |
| 0x06 | Compile Time | string |
| 0x07 | Free Memory | uint32 (bytes) |
| 0x08 | Device Role | uint8 (enum value) |
| 0x10 | AC Voltage | float32 (volts) |
| 0x11 | AC Current | int32 (milliamperes) |
| 0x12 | AC Frequency | float32 (hertz) |
| 0x13 | AC Power | int32 (milliwatts) |
| 0x50 | Status Flags | uint16 (bitfield) |

## Device Roles

The device role identifies the function of the device in the power system. The following roles are supported:

| Role Value | Enum Name | Description |
|------------|-----------|-------------|
| 0 | UNKNOWN | Default/unspecified role |
| 1 | GRID_INPUT | Grid power input measurement |
| 2 | INVERTER_AC_INPUT | Inverter AC input side |
| 3 | INVERTER_AC_OUTPUT | Inverter AC output side |
| 4 | INVERTER_DC_BATTERY | Inverter DC battery connection |
| 5 | INVERTER_DC_GRID_POWER | Inverter DC grid power |
| 6 | SINKER_AC_HEATER | AC load/heater device |
| 7 | SINKER_DC_HEATER | DC load/heater device |
| 8 | SINKER_AC_VEHICLE_CHARGER | AC vehicle charger |
| 9 | SINKER_DC_VEHICLE_CHARGER | DC vehicle charger |

All device role values are standardized in `powersync_tlv_format.h` and shared across projects for consistency.

## Configuration

```yaml
# Example configuration
powersync:
  id: my_powersync
  channel: 1                           # ESP-NOW channel (1-14)
  auto_add_peer: true                  # Auto-add broadcast peer
  broadcast_interval: 5s               # TLV broadcast interval
  system_update_interval: 100ms       # System info update interval
  firmware_version: "1.0.0"           # Required: firmware version
  device_role: GRID_INPUT              # Device role (see Device Roles below)
  simulate: false                      # Enable simulation mode for testing
  
  # Required hardware references
  rgb_power_enable: rgb_power          # Binary output for RGB power
  rgb_strip: status_led                # Light component for effects
  
  # Optional sensors (will be auto-updated with TLV data)
  ac_voltage_sensor:
    name: "AC Voltage"
  ac_current_sensor:
    name: "AC Current"
  ac_frequency_sensor:
    name: "AC Frequency"  
  ac_power_sensor:
    name: "AC Power"
  button_press_count_sensor:
    name: "Button Press Count"

# Required hardware components (examples)
output:
  - platform: gpio
    pin: GPIO2
    id: rgb_power

light:
  - platform: neopixel
    pin: GPIO8
    num_leds: 1
    id: status_led
    effects:
      - strobe:
          name: "Packet Received"
          colors:
            - state: true
              brightness: 90%
              red: 0%
              green: 100%
              blue: 0%
      - strobe:
          name: "Alert Red"
          colors:
            - state: true
              brightness: 100%
              red: 100%
              green: 0%
              blue: 0%
```

## API Methods

The component provides several public methods for external interaction:

```cpp
// Update AC measurements
powersync->set_ac_voltage(230.5);      // Set AC voltage in volts
powersync->set_ac_current(2.45);       // Set AC current in amperes  
powersync->set_ac_frequency(50.0);     // Set AC frequency in hertz
powersync->set_ac_power(564.25);       // Set AC power in watts

// Button interaction
powersync->increment_button_press_count();  // Increment button counter

// Manual broadcast trigger
powersync->trigger_broadcast();             // Force immediate broadcast
```

## Integration with Existing YAML

This component replaces the functionality found in `esp32_common_powersync.yaml`. The original YAML configuration included:

- ESP-NOW setup and event handlers
- TLV data construction and broadcasting
- Global variables for measurements
- Periodic system updates and broadcasting
- LED effect triggers

All of this functionality is now encapsulated in the C++ component with the same behavior and TLV packet structure.

## Advantages of the Component

1. **Performance**: Native C++ implementation is faster than YAML lambda functions
2. **Maintainability**: Centralized code in a reusable component
3. **Type Safety**: Strong typing prevents runtime errors
4. **Reusability**: Can be easily used across multiple projects
5. **Extensibility**: Easy to add new TLV types or modify behavior

## Dependencies

- ESP32 platform
- WiFi component (for MAC address and ESP-NOW)
- Output component (for RGB power control)
- Light component (for LED effects)

## Compatibility

This component maintains full compatibility with the original YAML configuration's TLV packet format, ensuring existing receivers continue to work without modification.