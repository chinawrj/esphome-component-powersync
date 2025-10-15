#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#ifdef USE_ESP32
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#endif

#include <cstdint>
#include <string>
#include <vector>

// Include standardized TLV format definitions
#include "powersync_tlv_format.h"

namespace esphome {
namespace powersync {

static const char *const TAG = "powersync";

// Device roles enum using standardized values from TLV format header
// Use different enum names to avoid macro conflicts
enum DeviceRole : uint8_t {
  ROLE_UNKNOWN = 0,
  ROLE_GRID_INPUT = 1,
  ROLE_INVERTER_AC_INPUT = 2,
  ROLE_INVERTER_AC_OUTPUT = 3,
  ROLE_INVERTER_DC_BATTERY = 4,
  ROLE_INVERTER_DC_GRID_POWER = 5,
  ROLE_SINKER_AC_HEATER = 6,
  ROLE_SINKER_DC_HEATER = 7,
  ROLE_SINKER_AC_VEHICLE_CHARGER = 8,
  ROLE_SINKER_DC_VEHICLE_CHARGER = 9,
  ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL = 10,
  
  // This must be the last entry - used to calculate array size
  ROLE_COUNT  // Automatically equals 11
};

// Status flags for TLV_TYPE_STATUS_FLAGS
enum StatusFlags : uint16_t {
  STATUS_POWER_ON = 0x0001,
  STATUS_ESP_NOW_ACTIVE = 0x0020
};

// Message queue types
enum MessageType : uint8_t {
  CMD_SEND_TLV = 0x01,
  DATA_TLV_RECEIVED = 0x02
};

// Maximum message body size (adjust as needed)
static const size_t MAX_MESSAGE_BODY_SIZE = 250;

// Maximum number of device roles - automatically calculated from enum
static const size_t MAX_DEVICE_ROLES = static_cast<size_t>(DeviceRole::ROLE_COUNT);

// Structure to store received device information
struct DeviceState {
  // Electrical measurements
  float voltage;           // AC Voltage in volts (V)
  float current;           // AC Current in amperes (A)
  float power;             // AC Power in watts (W)
  
  // Communication info
  int rssi;                // RSSI signal strength in dBm
  uint32_t uptime;         // Device uptime in seconds
  std::string firmware_version;  // Firmware version string
  
  // Timing
  uint32_t last_update_time;  // Timestamp of last update (millis())
  uint32_t data_age_ms;       // Age of data in milliseconds (updated periodically)
  
  // Status
  bool is_valid;           // Whether this device data is valid/active
  uint8_t src_addr[6];     // Source MAC address
  DeviceRole role;         // Device role type
  
  // Constructor to initialize with default values
  DeviceState() 
    : voltage(0.0f), current(0.0f), power(0.0f),
      rssi(0), uptime(0), firmware_version(""),
      last_update_time(0), data_age_ms(0), is_valid(false), role(ROLE_UNKNOWN) {
    memset(src_addr, 0, 6);
  }
};

// TLV parsing context structure for callback
struct TLVParseContext {
  DeviceState device_state;  // Temporary storage for parsed data
  bool has_voltage;
  bool has_current;
  bool has_power;
  bool has_uptime;
  bool has_firmware;
  bool has_role;
  
  TLVParseContext() 
    : has_voltage(false), has_current(false), has_power(false),
      has_uptime(false), has_firmware(false), has_role(false) {}
};

// Message structure for the queue
struct PowerSyncMessage {
  MessageType type;
  uint8_t body[MAX_MESSAGE_BODY_SIZE];
  size_t body_length;
  uint8_t src_addr[6];  // Source MAC address for received packets
  int rssi;             // RSSI for received packets
};

class PowerSyncComponent : public Component {
 public:
  PowerSyncComponent() = default;
  ~PowerSyncComponent();

  // Configuration setters
  void set_channel(uint8_t channel) { channel_ = channel; }
  void set_auto_add_peer(bool auto_add_peer) { auto_add_peer_ = auto_add_peer; }
  void set_broadcast_interval(uint32_t interval) { broadcast_interval_ = interval; }
  void set_system_update_interval(uint32_t interval) { system_update_interval_ = interval; }
  void set_power_decision_data_timeout(uint32_t timeout) { power_decision_data_timeout_ = timeout; }
  void set_firmware_version(const std::string &version) { firmware_version_ = version; }
  void set_device_role(DeviceRole role) { device_role_ = role; }

  // Optional sensor setters
  void set_ac_voltage_sensor(esphome::sensor::Sensor *sensor) { ac_voltage_sensor_ = sensor; }
  void set_ac_current_sensor(esphome::sensor::Sensor *sensor) { ac_current_sensor_ = sensor; }
  void set_ac_frequency_sensor(esphome::sensor::Sensor *sensor) { ac_frequency_sensor_ = sensor; }
  void set_ac_power_sensor(esphome::sensor::Sensor *sensor) { ac_power_sensor_ = sensor; }
  void set_button_press_count_sensor(esphome::sensor::Sensor *sensor) { button_press_count_sensor_ = sensor; }
  
  // DLT645 relay control binary sensor setter (for cross-module control)
  void set_dlt645_relay_trip_sensor(esphome::binary_sensor::BinarySensor *sensor) { dlt645_relay_trip_sensor_ = sensor; }

  // Component lifecycle
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  // Public methods for external access
  void increment_button_press_count();
  void set_ac_voltage(float voltage);
  void set_ac_current(float current);
  void set_ac_frequency(float frequency);
  void set_ac_power(float power);
  void trigger_broadcast();
  void send_tlv_command();
  
  // Device state management methods
  const DeviceState* get_device_state(DeviceRole role) const;
  bool is_device_active(DeviceRole role) const;
  void clear_inactive_devices();  // Remove devices that haven't updated recently

 protected:
  // Configuration
  uint8_t channel_ = 1;
  bool auto_add_peer_ = true;
  uint32_t broadcast_interval_ = 5000;  // 5 seconds
  uint32_t system_update_interval_ = 100;  // 100ms
  uint32_t power_decision_data_timeout_ = 60000;  // 60 seconds (default)

  // Optional sensors
  sensor::Sensor *ac_voltage_sensor_ = nullptr;
  sensor::Sensor *ac_current_sensor_ = nullptr;
  sensor::Sensor *ac_frequency_sensor_ = nullptr;
  sensor::Sensor *ac_power_sensor_ = nullptr;
  sensor::Sensor *button_press_count_sensor_ = nullptr;
  
  // DLT645 relay control binary sensor (for cross-module control)
  binary_sensor::BinarySensor *dlt645_relay_trip_sensor_ = nullptr;

  // Internal state
  bool espnow_ready_ = false;
  uint32_t press_count_ = 0;
  float tlv_ac_voltage_ = 0.0f;
  int32_t tlv_ac_current_ma_ = 0;
  float tlv_ac_frequency_ = 0.0f;
  int32_t tlv_ac_power_mw_ = 0;

  // System info (similar to esp32_system_info.yaml)
  std::string device_id_;
  std::string firmware_version_;
  std::string compile_time_;
  DeviceRole device_role_ = ROLE_UNKNOWN;
  uint8_t mac_address_bytes_[6];
  uint32_t uptime_seconds_ = 0;
  uint32_t free_memory_ = 0;

  // Device state array - stores information from other devices
  // Index corresponds to DeviceRole enum (0-9)
  // Only one device per role type is stored
  DeviceState device_states_[MAX_DEVICE_ROLES];
  
  // Device state timeout (milliseconds)
  static const uint32_t DEVICE_STATE_TIMEOUT = 30000;  // 30 seconds

  // Timing
  uint32_t last_broadcast_time_ = 0;
  uint32_t last_system_update_time_ = 0;

  // FreeRTOS task related
  TaskHandle_t espnow_task_handle_ = nullptr;
  static const uint32_t ESPNOW_TASK_STACK_SIZE = 4096;
  static const UBaseType_t ESPNOW_TASK_PRIORITY = 5;

  // Message queue related
  QueueHandle_t message_queue_ = nullptr;
  static const uint32_t MESSAGE_QUEUE_SIZE = 10;

  // ESP-NOW methods
  void init_espnow_();
  void setup_system_info_();
  void update_system_info_();
  std::vector<uint8_t> build_tlv_packet_();
  void broadcast_tlv_data_();
  void handle_packet_received_(const uint8_t *src_addr, const uint8_t *data, int len, int rssi);
  void handle_unknown_peer_(const uint8_t *src_addr, const uint8_t *data, int len, int rssi);
  void trigger_packet_received_effect_();
  void trigger_alert_red_effect_();
  
  // Device state management methods
  void update_device_state_(DeviceRole role, const uint8_t *src_addr, int rssi);
  void dump_device_states_table_();  // Dump device states in table format
  void update_all_device_data_age_();  // Update data age for all valid devices
  
  // Decision-making method based on network-wide device states
  void make_power_management_decisions_();

  // TLV helper methods
  void add_tlv_uptime_(std::vector<uint8_t> &payload);
  void add_tlv_device_id_(std::vector<uint8_t> &payload);
  void add_tlv_mac_address_(std::vector<uint8_t> &payload);
  void add_tlv_compile_time_(std::vector<uint8_t> &payload);
  void add_tlv_firmware_version_(std::vector<uint8_t> &payload);
  void add_tlv_ac_voltage_(std::vector<uint8_t> &payload);
  void add_tlv_ac_current_(std::vector<uint8_t> &payload);
  void add_tlv_ac_frequency_(std::vector<uint8_t> &payload);
  void add_tlv_ac_power_(std::vector<uint8_t> &payload);
  void add_tlv_device_role_(std::vector<uint8_t> &payload);
  void add_tlv_free_memory_(std::vector<uint8_t> &payload);
  void add_tlv_status_flags_(std::vector<uint8_t> &payload);

  // Utility methods
  void add_tlv_uint32_(std::vector<uint8_t> &payload, uint8_t type, uint32_t value);
  void add_tlv_int32_(std::vector<uint8_t> &payload, uint8_t type, int32_t value);
  void add_tlv_float_(std::vector<uint8_t> &payload, uint8_t type, float value);
  void add_tlv_string_(std::vector<uint8_t> &payload, uint8_t type, const std::string &value);
  void add_tlv_bytes_(std::vector<uint8_t> &payload, uint8_t type, const uint8_t *data, uint8_t len);

  // ESP-NOW callbacks (static functions)
  static void esp_now_send_cb_(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void esp_now_recv_cb_(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static PowerSyncComponent *instance_;

  // FreeRTOS task function
  static void espnow_task_function_(void *pvParameters);
  
  // TLV parsing callback (static function for use with tlv_parse_stream)
  static bool tlv_parse_callback_(uint8_t type, uint8_t length, const uint8_t* value, void* user_data);
};

}  // namespace powersync
}  // namespace esphome