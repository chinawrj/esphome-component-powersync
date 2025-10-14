#ifndef ESPHOME_TLV_FORMAT_H
#define ESPHOME_TLV_FORMAT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ESPHome TLV Data Format Specification
// Shared between ESPHome and ESP-IDF C projects
// This header provides constants, macros, and data structures for TLV parsing
// No function implementations are provided - developers should implement their own parsing logic

// TLV (Type-Length-Value) Format Overview:
// [Type (1 byte)] [Length (1 byte)] [Value (N bytes)]
// Total packet: [TLV1] [TLV2] ... [TLVn]

// IMPORTANT DATA FORMAT NOTES:
// - All multi-byte integers (uint16, uint32, int32) use BIG-ENDIAN byte order
// - All float32 values use IEEE 754 format in BIG-ENDIAN byte order
// - Fixed-point values (current/power) are stored as signed integers
// - String values are UTF-8 encoded without null terminator in TLV payload
// - MAC addresses are stored in network byte order (MSB first)

// Type Definitions (Shared Constants)
// These types must be kept in sync between ESPHome and ESP-IDF projects

// Basic Types (0x00-0x0F)
#define TLV_TYPE_UPTIME          0x01  // System uptime in seconds (uint32_t)
#define TLV_TYPE_TIMESTAMP       0x02  // Unix timestamp (uint32_t)
#define TLV_TYPE_DEVICE_ID       0x03  // Device identifier (string, max 32 bytes)
#define TLV_TYPE_FIRMWARE_VER    0x04  // Firmware version (string, max 32 bytes)
#define TLV_TYPE_MAC_ADDRESS     0x05  // Device MAC address (6 bytes)
#define TLV_TYPE_COMPILE_TIME    0x06  // Compile timestamp (string, max 32 bytes)
#define TLV_TYPE_FREE_MEMORY     0x07  // Free memory in bytes (uint32_t)
#define TLV_TYPE_DEVICE_ROLE     0x08  // Device role identifier (uint8_t)

// Electrical Measurements (0x10-0x2F)
// All electrical values follow DL/T 645-2007 smart meter protocol precision standards
#define TLV_TYPE_AC_VOLTAGE      0x10  // AC Voltage: float32, volts, 0.1V precision
#define TLV_TYPE_AC_CURRENT      0x11  // AC Current: int32_t, milliamperes, 0.001A precision (fixed-point)
#define TLV_TYPE_AC_FREQUENCY    0x12  // AC Frequency: float32, hertz, 0.01Hz precision  
#define TLV_TYPE_AC_POWER        0x13  // AC Power: int32_t, milliwatts, 0.001W precision (fixed-point)
#define TLV_TYPE_AC_POWER_FACTOR 0x14  // Power Factor: float32, dimensionless, 0.001 precision

// Energy Measurements (0x30-0x4F)
#define TLV_TYPE_ENERGY_TOTAL    0x30  // Total energy in kWh (float32)
#define TLV_TYPE_ENERGY_TODAY    0x31  // Today's energy in kWh (float32)

// Status and Flags (0x50-0x6F)
#define TLV_TYPE_STATUS_FLAGS    0x50  // Status flags (uint16_t)
#define TLV_TYPE_ERROR_CODE      0x51  // Error code (uint16_t)

// Environmental (0x70-0x8F)
#define TLV_TYPE_TEMPERATURE     0x70  // Temperature in Celsius (float32)
#define TLV_TYPE_HUMIDITY        0x71  // Humidity in % (float32)

// Custom/Extension (0xF0-0xFF)
#define TLV_TYPE_CUSTOM_START    0xF0  // Start of custom types

// Status Flags Bit Definitions
#define STATUS_FLAG_POWER_ON         0x0001  // Device powered on
#define STATUS_FLAG_CALIBRATING      0x0002  // Device is calibrating
#define STATUS_FLAG_ERROR            0x0004  // Error condition
#define STATUS_FLAG_LOW_BATTERY      0x0008  // Low battery warning
#define STATUS_FLAG_WIFI_CONNECTED   0x0010  // WiFi connected
#define STATUS_FLAG_ESP_NOW_ACTIVE   0x0020  // ESP-NOW active

// Error Codes
#define ERROR_NONE                   0x0000
#define ERROR_SENSOR_FAIL            0x0001
#define ERROR_COMMUNICATION_FAIL     0x0002
#define ERROR_CALIBRATION_FAIL       0x0003
#define ERROR_OVER_VOLTAGE           0x0004
#define ERROR_OVER_CURRENT           0x0005

// Device Role Values for TLV_TYPE_DEVICE_ROLE
#define DEVICE_ROLE_UNKNOWN                  0
#define DEVICE_ROLE_GRID_INPUT              1
#define DEVICE_ROLE_INVERTER_AC_INPUT       2
#define DEVICE_ROLE_INVERTER_AC_OUTPUT      3
#define DEVICE_ROLE_INVERTER_DC_BATTERY     4
#define DEVICE_ROLE_INVERTER_DC_GRID_POWER  5
#define DEVICE_ROLE_SINKER_AC_HEATER        6
#define DEVICE_ROLE_SINKER_DC_HEATER        7
#define DEVICE_ROLE_SINKER_AC_VEHICLE_CHARGER 8
#define DEVICE_ROLE_SINKER_DC_VEHICLE_CHARGER 9

// TLV Structure for reference
typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t value[];
} __attribute__((packed)) tlv_entry_t;

// TLV packet parser state structure (optional helper for implementers)
typedef struct {
    uint8_t *buffer;
    size_t buffer_size;
    size_t offset;
} tlv_parser_t;

// Helper macros for data size calculations
#define TLV_SIZE_UINT8    1
#define TLV_SIZE_UINT16   2
#define TLV_SIZE_UINT32   4
#define TLV_SIZE_INT32    4  // Added for fixed-point numbers
#define TLV_SIZE_FLOAT32  4
#define TLV_SIZE_STRING(len) (len)

// Fixed data sizes for specific TLV types
#define TLV_SIZE_UPTIME          4   // uint32_t, seconds
#define TLV_SIZE_TIMESTAMP       4   // uint32_t, unix timestamp
#define TLV_SIZE_MAC_ADDRESS     6   // 6 bytes MAC address
#define TLV_SIZE_COMPILE_TIME    32  // string, max 32 bytes
#define TLV_SIZE_FREE_MEMORY     4   // uint32_t, bytes
#define TLV_SIZE_DEVICE_ROLE     1   // uint8_t, device role enum value
#define TLV_SIZE_AC_VOLTAGE      4   // float32, volts
#define TLV_SIZE_AC_CURRENT      4   // int32_t, milliamperes (fixed-point)
#define TLV_SIZE_AC_FREQUENCY    4   // float32, hertz
#define TLV_SIZE_AC_POWER        4   // int32_t, milliwatts (fixed-point)
#define TLV_SIZE_AC_POWER_FACTOR 4   // float32, dimensionless
#define TLV_SIZE_ENERGY_TOTAL    4   // float32, kWh
#define TLV_SIZE_ENERGY_TODAY    4   // float32, kWh
#define TLV_SIZE_STATUS_FLAGS    2   // uint16_t, bit flags
#define TLV_SIZE_ERROR_CODE      2   // uint16_t, error code
#define TLV_SIZE_TEMPERATURE     4   // float32, celsius
#define TLV_SIZE_HUMIDITY        4   // float32, percentage

// Maximum sizes for variable-length types
#define TLV_MAX_DEVICE_ID_LEN     16
#define TLV_MAX_FIRMWARE_VER_LEN  16
#define TLV_MAX_COMPILE_TIME_LEN  32
#define TLV_MAX_STRING_LEN        64
#define TLV_MAC_ADDRESS_LEN       6

// Fixed-point conversion macros for DL/T 645-2007 compatibility
// Current: int32_t milliamperes <-> float amperes
#define TLV_CURRENT_MA_TO_A(ma)     ((float)(ma) / 1000.0f)
#define TLV_CURRENT_A_TO_MA(a)      ((int32_t)((a) * 1000.0f))

// Power: int32_t milliwatts <-> float watts  
#define TLV_POWER_MW_TO_W(mw)       ((float)(mw) / 1000.0f)
#define TLV_POWER_W_TO_MW(w)        ((int32_t)((w) * 1000.0f))

// Power: int32_t milliwatts <-> float kilowatts
#define TLV_POWER_MW_TO_KW(mw)      ((float)(mw) / 1000000.0f)
#define TLV_POWER_KW_TO_MW(kw)      ((int32_t)((kw) * 1000000.0f))

// Utility macros for TLV parsing and generation

// Get TLV entry total size including header
#define TLV_TOTAL_SIZE(length) (2 + (length))  // Type(1) + Length(1) + Value(length)

// Get pointer to next TLV entry in a buffer
#define TLV_NEXT_ENTRY(tlv_ptr) ((tlv_entry_t*)((uint8_t*)(tlv_ptr) + TLV_TOTAL_SIZE((tlv_ptr)->length)))

// Validate TLV entry fits within buffer bounds
#define TLV_VALIDATE_BOUNDS(tlv_ptr, buffer_start, buffer_size) \
    (((uint8_t*)(tlv_ptr) >= (uint8_t*)(buffer_start)) && \
     ((uint8_t*)(tlv_ptr) + TLV_TOTAL_SIZE((tlv_ptr)->length) <= (uint8_t*)(buffer_start) + (buffer_size)))

// Extract value from TLV entry
#define TLV_GET_VALUE_PTR(tlv_ptr) ((uint8_t*)&((tlv_ptr)->value[0]))

// Big-endian conversion macros (for cross-platform compatibility)
#define TLV_UINT16_FROM_BE(bytes) (((uint16_t)(bytes)[0] << 8) | (uint16_t)(bytes)[1])
#define TLV_UINT32_FROM_BE(bytes) (((uint32_t)(bytes)[0] << 24) | ((uint32_t)(bytes)[1] << 16) | \
                                   ((uint32_t)(bytes)[2] << 8) | (uint32_t)(bytes)[3])
#define TLV_INT32_FROM_BE(bytes)  ((int32_t)TLV_UINT32_FROM_BE(bytes))

#define TLV_UINT16_TO_BE(value, bytes) do { \
    (bytes)[0] = (uint8_t)((value) >> 8); \
    (bytes)[1] = (uint8_t)((value) & 0xFF); \
} while(0)

#define TLV_UINT32_TO_BE(value, bytes) do { \
    (bytes)[0] = (uint8_t)((value) >> 24); \
    (bytes)[1] = (uint8_t)((value) >> 16); \
    (bytes)[2] = (uint8_t)((value) >> 8); \
    (bytes)[3] = (uint8_t)((value) & 0xFF); \
} while(0)

#define TLV_INT32_TO_BE(value, bytes) TLV_UINT32_TO_BE((uint32_t)(value), bytes)

// Float32 conversion macros (IEEE 754 big-endian)
#define TLV_FLOAT32_FROM_BE(bytes, float_var) do { \
    union { float f; uint32_t u; } converter; \
    converter.u = TLV_UINT32_FROM_BE(bytes); \
    (float_var) = converter.f; \
} while(0)

#define TLV_FLOAT32_TO_BE(float_var, bytes) do { \
    union { float f; uint32_t u; } converter; \
    converter.f = (float_var); \
    TLV_UINT32_TO_BE(converter.u, bytes); \
} while(0)

/*
 * DETAILED TLV DATA FORMAT SPECIFICATIONS
 * =======================================
 * 
 * This section provides comprehensive format details for each TLV type
 * to ensure consistent parsing across different C platforms.
 */

// BASIC TYPES (0x00-0x0F)
//
// TLV_TYPE_UPTIME (0x01):
//   Format: uint32_t (4 bytes, big-endian)
//   Unit: seconds since device boot
//   Range: 0 to 4,294,967,295 seconds (~136 years)
//   Usage: Device uptime tracking
//
// TLV_TYPE_TIMESTAMP (0x02):
//   Format: uint32_t (4 bytes, big-endian)  
//   Unit: Unix timestamp (seconds since epoch)
//   Range: 0 to 2,147,483,647 (until year 2038)
//   Usage: Absolute time reference
//
// TLV_TYPE_DEVICE_ID (0x03):
//   Format: UTF-8 string (variable length, no null terminator)
//   Length: 1-16 bytes (as specified in Length field)
//   Content: Human-readable device identifier
//   Example: "M5NanoC6-C6-123456"
//
// TLV_TYPE_FIRMWARE_VER (0x04):
//   Format: UTF-8 string (variable length, no null terminator)
//   Length: 1-16 bytes (as specified in Length field)
//   Content: Firmware version string
//   Example: "v1.2.3" or "2024.1.1"
//
// TLV_TYPE_MAC_ADDRESS (0x05):
//   Format: 6-byte array (big-endian, network byte order)
//   Length: 6 bytes (fixed)
//   Content: IEEE 802.11 MAC address
//   Example: [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
//
// TLV_TYPE_COMPILE_TIME (0x06):
//   Format: UTF-8 string (variable length, no null terminator)
//   Length: 1-32 bytes (as specified in Length field)
//   Content: Compilation timestamp
//   Example: "2024-09-23 10:30:45" or "Sep 23 2024 10:30:45"
//   Note: Format may vary by compiler (__DATE__ __TIME__ macros)
//
// TLV_TYPE_FREE_MEMORY (0x07):
//   Format: uint32_t (4 bytes, big-endian)
//   Unit: Bytes
//   Range: 0 to 4,294,967,295 bytes (~4GB)
//   Usage: Available free memory in system heap
//   Note: Real-time memory usage monitoring
//
// TLV_TYPE_DEVICE_ROLE (0x08):
//   Format: uint8_t (1 byte)
//   Unit: Device role identifier (enum value)
//   Range: 0-9 (see DEVICE_ROLE_* constants)
//   Usage: Identifies the function/role of the device in power system
//   Values: 0=Unknown, 1=Grid Input, 2=Inverter AC Input, 3=Inverter AC Output,
//           4=Inverter DC Battery, 5=Inverter DC Grid Power, 6=AC Heater,
//           7=DC Heater, 8=AC Vehicle Charger, 9=DC Vehicle Charger

// ELECTRICAL MEASUREMENTS (0x10-0x2F)
//
// TLV_TYPE_AC_VOLTAGE (0x10):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Volts (V)
//   Precision: 0.1V (typical smart meter precision)
//   Range: 0.0 to 999.9V (typical residential/commercial range)
//   Usage: Line voltage measurement
//
// TLV_TYPE_AC_CURRENT (0x11):
//   Format: int32_t (4 bytes, big-endian, signed, fixed-point)
//   Unit: Milliamperes (mA)
//   Precision: 1mA (equivalent to 0.001A)
//   Range: -2,147,483,648 to 2,147,483,647 mA
//   Conversion: divide by 1000 to get amperes
//   Usage: Line current measurement (supports bidirectional flow)
//   Example: 2350 = 2.350A
//
// TLV_TYPE_AC_FREQUENCY (0x12):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Hertz (Hz)
//   Precision: 0.01Hz (typical smart meter precision)
//   Range: 40.00 to 70.00Hz (typical grid frequency range)
//   Usage: Grid frequency measurement
//
// TLV_TYPE_AC_POWER (0x13):
//   Format: int32_t (4 bytes, big-endian, signed, fixed-point)
//   Unit: Milliwatts (mW)
//   Precision: 1mW (equivalent to 0.001W)
//   Range: -2,147,483,648 to 2,147,483,647 mW
//   Conversion: divide by 1000 to get watts
//   Usage: Active power measurement (supports bidirectional flow)
//   Positive: consuming power, Negative: generating power
//   Example: 520000 = 520.000W
//
// TLV_TYPE_AC_POWER_FACTOR (0x14):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Dimensionless
//   Precision: 0.001 (typical smart meter precision)
//   Range: -1.000 to +1.000
//   Usage: Power factor measurement
//   Positive: leading, Negative: lagging

// ENERGY MEASUREMENTS (0x30-0x4F)
//
// TLV_TYPE_ENERGY_TOTAL (0x30):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Kilowatt-hours (kWh)
//   Precision: 0.01kWh (typical smart meter precision)
//   Range: 0.00 to 16,777,216.00 kWh
//   Usage: Cumulative energy consumption
//
// TLV_TYPE_ENERGY_TODAY (0x31):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Kilowatt-hours (kWh)
//   Precision: 0.01kWh
//   Range: 0.00 to 999.99 kWh (daily consumption)
//   Usage: Today's energy consumption

// STATUS AND FLAGS (0x50-0x6F)
//
// TLV_TYPE_STATUS_FLAGS (0x50):
//   Format: uint16_t (2 bytes, big-endian, bit field)
//   Bit definitions:
//     Bit 0 (0x0001): STATUS_FLAG_POWER_ON - Device powered on
//     Bit 1 (0x0002): STATUS_FLAG_CALIBRATING - Device calibrating
//     Bit 2 (0x0004): STATUS_FLAG_ERROR - Error condition present
//     Bit 3 (0x0008): STATUS_FLAG_LOW_BATTERY - Low battery warning
//     Bit 4 (0x0010): STATUS_FLAG_WIFI_CONNECTED - WiFi connected
//     Bit 5 (0x0020): STATUS_FLAG_ESP_NOW_ACTIVE - ESP-NOW active
//     Bits 6-15: Reserved for future use
//
// TLV_TYPE_ERROR_CODE (0x51):
//   Format: uint16_t (2 bytes, big-endian)
//   Values:
//     0x0000: ERROR_NONE - No error
//     0x0001: ERROR_SENSOR_FAIL - Sensor failure
//     0x0002: ERROR_COMMUNICATION_FAIL - Communication failure
//     0x0003: ERROR_CALIBRATION_FAIL - Calibration failure
//     0x0004: ERROR_OVER_VOLTAGE - Over voltage condition
//     0x0005: ERROR_OVER_CURRENT - Over current condition

// ENVIRONMENTAL (0x70-0x8F)
//
// TLV_TYPE_TEMPERATURE (0x70):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Degrees Celsius (°C)
//   Precision: 0.1°C
//   Range: -40.0 to +125.0°C (typical sensor range)
//   Usage: Ambient temperature measurement
//
// TLV_TYPE_HUMIDITY (0x71):
//   Format: IEEE 754 float32 (4 bytes, big-endian)
//   Unit: Relative humidity percentage (%)
//   Precision: 0.1%
//   Range: 0.0 to 100.0%
//   Usage: Ambient humidity measurement

// IMPLEMENTATION EXAMPLES
// =======================
//
// Example 1: Parse a TLV packet
//
//   uint8_t packet[] = { 0x01, 0x04, 0x00, 0x00, 0x12, 0x34,  // UPTIME TLV
//                        0x10, 0x04, 0x43, 0x5C, 0x66, 0x66 }; // VOLTAGE TLV
//   
//   tlv_entry_t *tlv = (tlv_entry_t*)packet;
//   while ((uint8_t*)tlv < packet + sizeof(packet)) {
//       if (tlv->type == TLV_TYPE_UPTIME && tlv->length == 4) {
//           uint32_t uptime = TLV_UINT32_FROM_BE(tlv->value);
//           printf("Uptime: %u seconds\n", uptime);
//       } else if (tlv->type == TLV_TYPE_AC_VOLTAGE && tlv->length == 4) {
//           float voltage;
//           TLV_FLOAT32_FROM_BE(tlv->value, voltage);
//           printf("Voltage: %.1f V\n", voltage);
//       }
//       tlv = TLV_NEXT_ENTRY(tlv);
//   }
//
// Example 2: Build a TLV packet
//
//   uint8_t buffer[64];
//   size_t offset = 0;
//   
//   // Add uptime TLV
//   buffer[offset++] = TLV_TYPE_UPTIME;
//   buffer[offset++] = 4;
//   TLV_UINT32_TO_BE(12345, &buffer[offset]);
//   offset += 4;
//   
//   // Add current TLV (fixed-point)
//   buffer[offset++] = TLV_TYPE_AC_CURRENT;
//   buffer[offset++] = 4;
//   int32_t current_ma = TLV_CURRENT_A_TO_MA(2.350f);  // 2.350A -> 2350mA
//   TLV_INT32_TO_BE(current_ma, &buffer[offset]);
//   offset += 4;
//   
//   // Add compile time TLV
//   const char* compile_time = __DATE__ " " __TIME__;  // "Sep 23 2024 10:30:45"
//   buffer[offset++] = TLV_TYPE_COMPILE_TIME;
//   buffer[offset++] = strlen(compile_time);
//   memcpy(&buffer[offset], compile_time, strlen(compile_time));
//   offset += strlen(compile_time);

// BYTE ORDER EXAMPLES
// ===================
//
// Example 1: uint32_t value 0x12345678 (305419896 decimal)
//   Bytes: [0x12, 0x34, 0x56, 0x78] (big-endian)
//
// Example 2: int32_t value -123456 (0xFFFE1DC0)
//   Bytes: [0xFF, 0xFE, 0x1D, 0xC0] (big-endian, two's complement)
//
// Example 3: float32 value 123.456 (0x42F6E979)
//   Bytes: [0x42, 0xF6, 0xE9, 0x79] (big-endian, IEEE 754)
//
// Example 4: MAC address 12:34:56:78:9A:BC
//   Bytes: [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC] (network byte order)

// FIXED-POINT ARITHMETIC EXAMPLES
// ================================
//
// Current conversion examples:
//   2.350A -> 2350mA (int32_t) -> TLV: [0x00, 0x00, 0x09, 0x2E]
//   -1.5A -> -1500mA (int32_t) -> TLV: [0xFF, 0xFF, 0xFA, 0x24]
//
// Power conversion examples:
//   520.0W -> 520000mW (int32_t) -> TLV: [0x00, 0x07, 0xEF, 0x40]  
//   -100.5W -> -100500mW (int32_t) -> TLV: [0xFF, 0xFE, 0x76, 0xDC]

#endif // ESPHOME_TLV_FORMAT_H