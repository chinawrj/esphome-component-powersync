#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/output/binary_output.h"

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
  ROLE_SINKER_DC_VEHICLE_CHARGER = 9
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
  void set_firmware_version(const std::string &version) { firmware_version_ = version; }
  void set_device_role(DeviceRole role) { device_role_ = role; }
  void set_rgb_power_enable(esphome::output::BinaryOutput *output) { rgb_power_enable_ = output; }
  void set_rgb_strip(esphome::light::LightState *light) { rgb_strip_ = light; }
  void set_simulate(bool simulate) { simulate_ = simulate; }

  // Optional sensor setters
  void set_ac_voltage_sensor(esphome::sensor::Sensor *sensor) { ac_voltage_sensor_ = sensor; }
  void set_ac_current_sensor(esphome::sensor::Sensor *sensor) { ac_current_sensor_ = sensor; }
  void set_ac_frequency_sensor(esphome::sensor::Sensor *sensor) { ac_frequency_sensor_ = sensor; }
  void set_ac_power_sensor(esphome::sensor::Sensor *sensor) { ac_power_sensor_ = sensor; }
  void set_button_press_count_sensor(esphome::sensor::Sensor *sensor) { button_press_count_sensor_ = sensor; }

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

 protected:
  // Configuration
  uint8_t channel_ = 1;
  bool auto_add_peer_ = true;
  uint32_t broadcast_interval_ = 5000;  // 5 seconds
  uint32_t system_update_interval_ = 100;  // 100ms
  bool simulate_ = false;

  // Hardware references
  output::BinaryOutput *rgb_power_enable_ = nullptr;
  light::LightState *rgb_strip_ = nullptr;

  // Optional sensors
  sensor::Sensor *ac_voltage_sensor_ = nullptr;
  sensor::Sensor *ac_current_sensor_ = nullptr;
  sensor::Sensor *ac_frequency_sensor_ = nullptr;
  sensor::Sensor *ac_power_sensor_ = nullptr;
  sensor::Sensor *button_press_count_sensor_ = nullptr;

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
  
  // Simulation method
  void simulate_ac_measurements_();

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
};

}  // namespace powersync
}  // namespace esphome