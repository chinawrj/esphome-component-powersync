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
#define DEVICE_ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL 10

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
//   Range: 0-10 (see DEVICE_ROLE_* constants)
//   Usage: Identifies the function/role of the device in power system
//   Values: 0=Unknown, 1=Grid Input, 2=Inverter AC Input, 3=Inverter AC Output,
//           4=Inverter DC Battery, 5=Inverter DC Grid Power, 6=AC Heater,
//           7=DC Heater, 8=AC Vehicle Charger, 9=DC Vehicle Charger,
//           10=Solar Inverter Output Total

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

// ============================================================================
// TLV Stream Parsing API (Pure C, compatible with C and C++ projects)
// ============================================================================
// These functions provide efficient zero-copy streaming TLV packet parsing

/**
 * @brief TLV parse callback function type
 * 
 * This callback is invoked for each TLV entry found during stream parsing.
 * 
 * @param type TLV type identifier (see TLV_TYPE_* constants)
 * @param length Length of the value field in bytes
 * @param value Pointer to the value data (not null-terminated for strings)
 * @param user_data User-provided context pointer
 * @return true to continue parsing, false to stop parsing
 * 
 * Example callback implementation:
 * @code
 * bool my_callback(uint8_t type, uint8_t length, const uint8_t* value, void* user_data) {
 *     switch(type) {
 *         case TLV_TYPE_AC_VOLTAGE: {
 *             float voltage;
 *             TLV_FLOAT32_FROM_BE(value, voltage);
 *             printf("Voltage: %.1f V\n", voltage);
 *             break;
 *         }
 *         case TLV_TYPE_DEVICE_ID: {
 *             char device_id[33] = {0};
 *             memcpy(device_id, value, length);
 *             printf("Device: %s\n", device_id);
 *             break;
 *         }
 *     }
 *     return true; // Continue parsing
 * }
 * @endcode
 */
typedef bool (*tlv_parse_callback_t)(uint8_t type, uint8_t length, const uint8_t* value, void* user_data);

/**
 * @brief Parse TLV packet stream using callback function
 * 
 * This function provides efficient zero-copy streaming parsing of TLV packets.
 * It validates packet structure and invokes the callback for each valid TLV entry.
 * 
 * Features:
 * - Zero memory allocation (zero-copy)
 * - Automatic boundary checking
 * - Malformed packet detection
 * - Early termination support via callback return value
 * 
 * @param buffer Pointer to the TLV packet buffer
 * @param buffer_size Size of the buffer in bytes
 * @param callback Callback function to process each TLV entry
 * @param user_data User-provided context pointer passed to callback
 * @return Number of successfully parsed TLV entries, or negative error code:
 *         -1: Invalid parameters (NULL buffer or callback)
 *         -2: Malformed packet (incomplete TLV entry)
 *         -3: Callback requested early termination
 * 
 * Usage example:
 * @code
 * uint8_t packet[] = { 0x01, 0x04, 0x00, 0x00, 0x12, 0x34,  // UPTIME TLV
 *                      0x10, 0x04, 0x43, 0x5C, 0x66, 0x66 }; // VOLTAGE TLV
 * int result = tlv_parse_stream(packet, sizeof(packet), my_callback, NULL);
 * if (result >= 0) {
 *     printf("Parsed %d TLV entries\n", result);
 * } else {
 *     printf("Parse error: %d\n", result);
 * }
 * @endcode
 */
static inline int tlv_parse_stream(const uint8_t* buffer, size_t buffer_size, 
                                   tlv_parse_callback_t callback, void* user_data) {
    // Validate input parameters
    if (buffer == NULL || callback == NULL || buffer_size == 0) {
        return -1; // Invalid parameters
    }
    
    size_t offset = 0;
    int parsed_count = 0;
    
    while (offset < buffer_size) {
        // Check if we have at least Type and Length bytes
        if (offset + 2 > buffer_size) {
            return -2; // Malformed packet: incomplete TLV header
        }
        
        uint8_t type = buffer[offset];
        uint8_t length = buffer[offset + 1];
        
        // Check if the value field fits within the buffer
        if (offset + 2 + length > buffer_size) {
            return -2; // Malformed packet: value field exceeds buffer
        }
        
        // Get pointer to value data
        const uint8_t* value = (length > 0) ? &buffer[offset + 2] : NULL;
        
        // Invoke callback
        bool continue_parsing = callback(type, length, value, user_data);
        if (!continue_parsing) {
            return -3; // Callback requested early termination
        }
        
        // Move to next TLV entry
        offset += 2 + length;
        parsed_count++;
    }
    
    return parsed_count; // Return number of successfully parsed entries
}

/**
 * @brief Validate TLV packet structure without parsing
 * 
 * This function performs a quick validation of TLV packet structure
 * without invoking any callbacks or processing the data.
 * 
 * @param buffer Pointer to the TLV packet buffer
 * @param buffer_size Size of the buffer in bytes
 * @return true if packet structure is valid, false otherwise
 * 
 * Usage example:
 * @code
 * if (tlv_validate_packet(data, len)) {
 *     printf("Valid TLV packet\n");
 * } else {
 *     printf("Invalid TLV packet structure\n");
 * }
 * @endcode
 */
static inline bool tlv_validate_packet(const uint8_t* buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return false;
    }
    
    size_t offset = 0;
    
    while (offset < buffer_size) {
        // Check for TLV header
        if (offset + 2 > buffer_size) {
            return false; // Incomplete header
        }
        
        uint8_t length = buffer[offset + 1];
        
        // Check if value field fits
        if (offset + 2 + length > buffer_size) {
            return false; // Value exceeds buffer
        }
        
        offset += 2 + length;
    }
    
    return true; // All TLV entries are valid
}

/**
 * @brief Count number of TLV entries in a packet
 * 
 * @param buffer Pointer to the TLV packet buffer
 * @param buffer_size Size of the buffer in bytes
 * @return Number of TLV entries, or -1 on error
 */
static inline int tlv_count_entries(const uint8_t* buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return -1;
    }
    
    size_t offset = 0;
    int count = 0;
    
    while (offset < buffer_size) {
        if (offset + 2 > buffer_size) {
            return -1; // Malformed packet
        }
        
        uint8_t length = buffer[offset + 1];
        
        if (offset + 2 + length > buffer_size) {
            return -1; // Malformed packet
        }
        
        offset += 2 + length;
        count++;
    }
    
    return count;
}

// ============================================================================
// TLV Helper Functions (Pure C, compatible with C and C++ projects)
// ============================================================================
// These functions are placed at the end to ensure all constants are defined

// Helper function to convert device role value to string
// Pure C function compatible with both C and C++ projects
// Usage: const char* name = tlv_device_role_to_string(role_value);
static inline const char* tlv_device_role_to_string(uint8_t role) {
    switch (role) {
        case DEVICE_ROLE_UNKNOWN: return "UNKNOWN";
        case DEVICE_ROLE_GRID_INPUT: return "GRID_INPUT";
        case DEVICE_ROLE_INVERTER_AC_INPUT: return "INVERTER_AC_INPUT";
        case DEVICE_ROLE_INVERTER_AC_OUTPUT: return "INVERTER_AC_OUTPUT";
        case DEVICE_ROLE_INVERTER_DC_BATTERY: return "INVERTER_DC_BATTERY";
        case DEVICE_ROLE_INVERTER_DC_GRID_POWER: return "INVERTER_DC_GRID_POWER";
        case DEVICE_ROLE_SINKER_AC_HEATER: return "SINKER_AC_HEATER";
        case DEVICE_ROLE_SINKER_DC_HEATER: return "SINKER_DC_HEATER";
        case DEVICE_ROLE_SINKER_AC_VEHICLE_CHARGER: return "SINKER_AC_VEHICLE_CHARGER";
        case DEVICE_ROLE_SINKER_DC_VEHICLE_CHARGER: return "SINKER_DC_VEHICLE_CHARGER";
        case DEVICE_ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL: return "SOLOAR_INVERTER_OUTPUT_TOTAL";
        default: return "INVALID";
    }
}

// Helper function to convert TLV type value to string
// Usage: const char* name = tlv_type_to_string(0x01);
static inline const char* tlv_type_to_string(uint8_t type) {
    switch (type) {
        // Basic Types (0x00-0x0F)
        case TLV_TYPE_UPTIME: return "UPTIME";
        case TLV_TYPE_TIMESTAMP: return "TIMESTAMP";
        case TLV_TYPE_DEVICE_ID: return "DEVICE_ID";
        case TLV_TYPE_FIRMWARE_VER: return "FIRMWARE_VER";
        case TLV_TYPE_MAC_ADDRESS: return "MAC_ADDRESS";
        case TLV_TYPE_COMPILE_TIME: return "COMPILE_TIME";
        case TLV_TYPE_FREE_MEMORY: return "FREE_MEMORY";
        case TLV_TYPE_DEVICE_ROLE: return "DEVICE_ROLE";
        
        // Electrical Measurements (0x10-0x2F)
        case TLV_TYPE_AC_VOLTAGE: return "AC_VOLTAGE";
        case TLV_TYPE_AC_CURRENT: return "AC_CURRENT";
        case TLV_TYPE_AC_FREQUENCY: return "AC_FREQUENCY";
        case TLV_TYPE_AC_POWER: return "AC_POWER";
        case TLV_TYPE_AC_POWER_FACTOR: return "AC_POWER_FACTOR";
        
        // Energy Measurements (0x30-0x4F)
        case TLV_TYPE_ENERGY_TOTAL: return "ENERGY_TOTAL";
        case TLV_TYPE_ENERGY_TODAY: return "ENERGY_TODAY";
        
        // Status and Flags (0x50-0x6F)
        case TLV_TYPE_STATUS_FLAGS: return "STATUS_FLAGS";
        case TLV_TYPE_ERROR_CODE: return "ERROR_CODE";
        
        // Environmental (0x70-0x8F)
        case TLV_TYPE_TEMPERATURE: return "TEMPERATURE";
        case TLV_TYPE_HUMIDITY: return "HUMIDITY";
        
        default: 
            return (type >= TLV_TYPE_CUSTOM_START) ? "CUSTOM" : "UNKNOWN";
    }
}

// Helper function to get expected data size for a TLV type
// Returns 0 for variable-length types (strings)
// Usage: uint8_t size = tlv_type_expected_size(TLV_TYPE_UPTIME);
static inline uint8_t tlv_type_expected_size(uint8_t type) {
    switch (type) {
        case TLV_TYPE_UPTIME: return TLV_SIZE_UPTIME;
        case TLV_TYPE_TIMESTAMP: return TLV_SIZE_TIMESTAMP;
        case TLV_TYPE_DEVICE_ID: return 0; // Variable length
        case TLV_TYPE_FIRMWARE_VER: return 0; // Variable length
        case TLV_TYPE_MAC_ADDRESS: return TLV_SIZE_MAC_ADDRESS;
        case TLV_TYPE_COMPILE_TIME: return 0; // Variable length
        case TLV_TYPE_FREE_MEMORY: return TLV_SIZE_FREE_MEMORY;
        case TLV_TYPE_DEVICE_ROLE: return TLV_SIZE_DEVICE_ROLE;
        case TLV_TYPE_AC_VOLTAGE: return TLV_SIZE_AC_VOLTAGE;
        case TLV_TYPE_AC_CURRENT: return TLV_SIZE_AC_CURRENT;
        case TLV_TYPE_AC_FREQUENCY: return TLV_SIZE_AC_FREQUENCY;
        case TLV_TYPE_AC_POWER: return TLV_SIZE_AC_POWER;
        case TLV_TYPE_AC_POWER_FACTOR: return TLV_SIZE_AC_POWER_FACTOR;
        case TLV_TYPE_ENERGY_TOTAL: return TLV_SIZE_ENERGY_TOTAL;
        case TLV_TYPE_ENERGY_TODAY: return TLV_SIZE_ENERGY_TODAY;
        case TLV_TYPE_STATUS_FLAGS: return TLV_SIZE_STATUS_FLAGS;
        case TLV_TYPE_ERROR_CODE: return TLV_SIZE_ERROR_CODE;
        case TLV_TYPE_TEMPERATURE: return TLV_SIZE_TEMPERATURE;
        case TLV_TYPE_HUMIDITY: return TLV_SIZE_HUMIDITY;
        default: return 0; // Unknown or variable length
    }
}

#endif // ESPHOME_TLV_FORMAT_H