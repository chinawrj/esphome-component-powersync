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

## TODO

- [ ] Get current relay state

## DL/T 645-2007 Relay Control Data Identifiers

### Overview
DL/T 645-2007 protocol supports remote relay control for smart meters, enabling operations like power cut-off, reconnection, and status queries. The following data identifiers are used for relay operations.

### Relay Control Commands

#### Control Data Identifiers
```cpp
// DL/T 645-2007 Relay Control Data Identifiers
#define DI_RELAY_CONTROL      0x1C010000  // 继电器控制 (Relay Control)
#define DI_RELAY_STATUS       0x1C020000  // 继电器状态查询 (Relay Status Query)
```

#### Control Codes (Data Field Values)
When sending relay control commands using `DI_RELAY_CONTROL (0x1C010000)`:

| Control Code | Description | Operation |
|--------------|-------------|-----------|
| `0x1A` | Trip/Cut-off | 断开继电器 (Open relay, cut power) |
| `0x1B` | Close/Connect | 闭合继电器 (Close relay, restore power) |

#### Status Query Response
When querying relay status using `DI_RELAY_STATUS (0x1C020000)`:

| Status Code | Description | Meaning |
|-------------|-------------|---------|
| `0x00` | Relay Closed | 继电器闭合 (Relay is closed, power connected) |
| `0x01` | Relay Open | 继电器断开 (Relay is open, power cut off) |
| `0x02` | Fault State | 故障状态 (Relay in fault condition) |

### Command Frame Examples

#### Example 1: Send Relay Trip Command
```cpp
// Command to open relay (cut power)
uint8_t trip_command[] = {
    0xFE, 0xFE, 0xFE, 0xFE,           // Preamble (optional)
    0x68,                              // Start delimiter
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // Address (broadcast or specific meter)
    0x68,                              // Start delimiter
    0x14,                              // Control code (Write command)
    0x05,                              // Data length (5 bytes: 4 for DI + 1 for control code)
    // Data field (must be scrambled with +0x33)
    0x33, 0x34, 0x34, 0x4F,           // DI: 0x1C010000 + 0x33333333
    0x4D,                              // Control code: 0x1A + 0x33 = 0x4D (Trip)
    0xCS,                              // Checksum (calculated)
    0x16                               // End delimiter
};
```

#### Example 2: Send Relay Close Command
```cpp
// Command to close relay (restore power)
uint8_t close_command[] = {
    0xFE, 0xFE, 0xFE, 0xFE,           // Preamble (optional)
    0x68,                              // Start delimiter
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // Address
    0x68,                              // Start delimiter
    0x14,                              // Control code (Write command)
    0x05,                              // Data length
    // Data field (must be scrambled with +0x33)
    0x33, 0x34, 0x34, 0x4F,           // DI: 0x1C010000 + 0x33333333
    0x4E,                              // Control code: 0x1B + 0x33 = 0x4E (Close)
    0xCS,                              // Checksum (calculated)
    0x16                               // End delimiter
};
```

#### Example 3: Query Relay Status
```cpp
// Command to query relay status
uint8_t query_command[] = {
    0xFE, 0xFE, 0xFE, 0xFE,           // Preamble (optional)
    0x68,                              // Start delimiter
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // Address
    0x68,                              // Start delimiter
    0x11,                              // Control code (Read command)
    0x04,                              // Data length (4 bytes for DI)
    // Data field (must be scrambled with +0x33)
    0x33, 0x34, 0x35, 0x4F,           // DI: 0x1C020000 + 0x33333333
    0xCS,                              // Checksum (calculated)
    0x16                               // End delimiter
};

// Expected response format
// Response data field (after descrambling -0x33):
// [DI: 4 bytes] [Status: 1 byte]
// Status byte: 0x00 (Closed), 0x01 (Open), 0x02 (Fault)
```

### ESPHome Integration Example

```yaml
# ESPHome configuration for relay control via DL/T 645-2007
esphome:
  name: meter-relay-control
  
uart:
  id: meter_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 2400
  parity: EVEN

switch:
  - platform: template
    name: "Meter Relay"
    id: relay_switch
    optimistic: false
    turn_on_action:
      - lambda: |-
          // Send relay close command (0x1B)
          std::vector<uint8_t> cmd = {
              0xFE, 0xFE, 0x68,
              0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
              0x68, 0x14, 0x05,
              0x33, 0x34, 0x34, 0x4F, 0x4E  // DI + 0x1B+0x33
          };
          // Calculate checksum
          uint8_t cs = 0;
          for(int i=2; i<cmd.size(); i++) cs += cmd[i];
          cmd.push_back(cs);
          cmd.push_back(0x16);
          id(meter_uart).write_array(cmd.data(), cmd.size());
          
    turn_off_action:
      - lambda: |-
          // Send relay trip command (0x1A)
          std::vector<uint8_t> cmd = {
              0xFE, 0xFE, 0x68,
              0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
              0x68, 0x14, 0x05,
              0x33, 0x34, 0x34, 0x4F, 0x4D  // DI + 0x1A+0x33
          };
          uint8_t cs = 0;
          for(int i=2; i<cmd.size(); i++) cs += cmd[i];
          cmd.push_back(cs);
          cmd.push_back(0x16);
          id(meter_uart).write_array(cmd.data(), cmd.size());

binary_sensor:
  - platform: template
    name: "Relay Status"
    id: relay_status
    
interval:
  - interval: 30s
    then:
      - lambda: |-
          // Query relay status
          std::vector<uint8_t> cmd = {
              0xFE, 0xFE, 0x68,
              0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
              0x68, 0x11, 0x04,
              0x33, 0x34, 0x35, 0x4F  // DI: 0x1C020000 + 0x33
          };
          uint8_t cs = 0;
          for(int i=2; i<cmd.size(); i++) cs += cmd[i];
          cmd.push_back(cs);
          cmd.push_back(0x16);
          id(meter_uart).write_array(cmd.data(), cmd.size());
          
          // Wait and read response
          delay(100);
          std::vector<uint8_t> response;
          while(id(meter_uart).available()) {
              response.push_back(id(meter_uart).read());
          }
          
          // Parse status (simplified - needs full frame validation)
          if(response.size() > 15) {
              uint8_t status = response[response.size()-3] - 0x33; // Descramble
              id(relay_status).publish_state(status == 0x00); // true=closed
          }
```

### Important Notes

1. **Authentication Required**: Most modern smart meters require authentication (password) before accepting relay control commands. The authentication process involves:
   - Sending authentication command with password (typically 8 bytes)
   - Data identifier: `0x04000100` for password verification
   - Successful authentication allows subsequent control commands

2. **Security Considerations**:
   - Relay control is a privileged operation
   - Default passwords should be changed immediately
   - Implement rate limiting to prevent abuse
   - Log all relay control operations

3. **Response Validation**:
   - Always verify control code in response (`0x94` for successful write)
   - Check for error responses (`0xD4` or `0xB4`)
   - Implement timeout handling (typical: 500ms - 2s)

4. **Compatibility**:
   - Not all meters support remote relay control
   - Some meters may use manufacturer-specific extensions
   - Verify meter capabilities before implementing control

### Reference
- **Standard**: DL/T 645-2007 "Multi-function Watt-hour Meter Communication Protocol"
- **Related Sections**: 
  - Section 6.3.4: Remote control commands
  - Section 7.3: Status word definitions
  - Appendix A: Data identifier table