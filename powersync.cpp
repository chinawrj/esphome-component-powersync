#include "powersync.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <cmath>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <string.h>
#include <algorithm>
#include <array>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

namespace esphome
{
namespace powersync
{

// Grid feed state constants (three-state logic)
static constexpr int8_t GRID_FEED_STATE_INVALID = -1;  // Initial state (unknown)
static constexpr int8_t GRID_FEED_STATE_NORMAL = 0;    // Normal state (power >= 0)
static constexpr int8_t GRID_FEED_STATE_FEEDING = 1;   // Reverse power state (power < 0)

PowerSyncComponent *PowerSyncComponent::instance_ = nullptr;

PowerSyncComponent::~PowerSyncComponent()
{
    // Clean up message queue
    if (this->message_queue_ != nullptr) {
        vQueueDelete(this->message_queue_);
        this->message_queue_ = nullptr;
    }
    
    // Clean up ESP-NOW task
    if (this->espnow_task_handle_ != nullptr) {
        vTaskDelete(this->espnow_task_handle_);
        this->espnow_task_handle_ = nullptr;
    }
    
    // Deinitialize ESP-NOW
    if (this->espnow_ready_) {
        esp_now_deinit();
        this->espnow_ready_ = false;
    }
}

void PowerSyncComponent::setup()
{
    ESP_LOGCONFIG(TAG, "Setting up PowerSync component...");

    // Set static instance for callbacks
    instance_ = this;

    // Initialize system info
    this->setup_system_info_();

    // Initialize ESP-NOW
    this->init_espnow_();

    // Create message queue for communication with ESP-NOW task
    this->message_queue_ = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(PowerSyncMessage));
    if (this->message_queue_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create message queue");
        return;
    }
    ESP_LOGI(TAG, "Message queue created successfully");

    // Create and start the ESP-NOW dedicated task
    BaseType_t result = xTaskCreate(
        espnow_task_function_,                    // Task function
        "PSYNC",                                  // Task name
        ESPNOW_TASK_STACK_SIZE,                   // Stack size
        this,                                     // Parameters passed to task
        ESPNOW_TASK_PRIORITY,                     // Priority
        &this->espnow_task_handle_               // Task handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP-NOW task");
        return;
    }
    
    ESP_LOGCONFIG(TAG, "PowerSync component setup completed");
    ESP_LOGCONFIG(TAG, "  Channel: %d", this->channel_);
    ESP_LOGCONFIG(TAG, "  Auto add peer: %s", this->auto_add_peer_ ? "YES" : "NO");
    ESP_LOGCONFIG(TAG, "  Broadcast interval: %lu ms", this->broadcast_interval_);
    ESP_LOGCONFIG(TAG, "  System update interval: %lu ms", this->system_update_interval_);
    ESP_LOGCONFIG(TAG, "  Power decision data timeout: %lu ms", this->power_decision_data_timeout_);
    ESP_LOGCONFIG(TAG, "  ESP-NOW task created with priority: %d", ESPNOW_TASK_PRIORITY);
}

void PowerSyncComponent::loop()
{
    // ESP-NOW operations are now handled by a dedicated FreeRTOS task
    // This loop method is kept minimal for any future non-ESP-NOW operations
    // The dedicated task handles:
    // - System info updates at system_update_interval_ 
    // - TLV data broadcasting at broadcast_interval_
}

void PowerSyncComponent::init_espnow_()
{
    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    // Initialize ESP-NOW
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(result));
        return;
    }

    // Set WiFi channel
    esp_wifi_set_channel(this->channel_, WIFI_SECOND_CHAN_NONE);
    ESP_LOGI(TAG, "WiFi channel set to: %d", this->channel_);

    // Register callbacks
    esp_now_register_send_cb(PowerSyncComponent::esp_now_send_cb_);
    esp_now_register_recv_cb(PowerSyncComponent::esp_now_recv_cb_);

    // Add broadcast peer if auto_add_peer is enabled
    if (this->auto_add_peer_) {
        esp_now_peer_info_t peer_info = {};
        uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        memcpy(peer_info.peer_addr, broadcast_addr, 6);
        peer_info.channel = this->channel_;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false; // No encryption for broadcast

        result = esp_now_add_peer(&peer_info);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "Broadcast peer added successfully");
        }
    }

    this->espnow_ready_ = true;
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
}

void PowerSyncComponent::esp_now_send_cb_(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (instance_ == nullptr) {
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "ESP-NOW send success");
    } else {
        ESP_LOGW(TAG, "ESP-NOW send failed");
    }
}

void PowerSyncComponent::esp_now_recv_cb_(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (instance_ == nullptr) {
        return;
    }

    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool is_broadcast = (memcmp(info->des_addr, broadcast_addr, 6) == 0);

    if (is_broadcast) {
        instance_->handle_packet_received_(info->src_addr, data, len, info->rx_ctrl->rssi);
    } else {
        instance_->handle_unknown_peer_(info->src_addr, data, len, info->rx_ctrl->rssi);
    }
}

void PowerSyncComponent::setup_system_info_()
{
    // Device ID (similar to esp32_system_info.yaml)
    this->device_id_ = App.get_name();

    // Firmware version is now set from configuration, no default value here
    // this->firmware_version_ is set via set_firmware_version() from YAML config

    // Compile time
    this->compile_time_ = App.get_compilation_time();

    // MAC address
    esp_read_mac(this->mac_address_bytes_, ESP_MAC_WIFI_STA);

    ESP_LOGI(TAG, "System info initialized:");
    ESP_LOGI(TAG, "  Device ID: %s", this->device_id_.c_str());
    ESP_LOGI(TAG, "  Firmware: %s", this->firmware_version_.c_str());
    ESP_LOGI(TAG, "  Compile time: %s", this->compile_time_.c_str());
    ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             this->mac_address_bytes_[0], this->mac_address_bytes_[1], this->mac_address_bytes_[2],
             this->mac_address_bytes_[3], this->mac_address_bytes_[4], this->mac_address_bytes_[5]);
    
    // Initialize current device's state in device_states_ array
    if (this->device_role_ < MAX_DEVICE_ROLES) {
        DeviceState& my_state = this->device_states_[this->device_role_];
        my_state.role = this->device_role_;
        my_state.is_valid = true;
        my_state.last_update_time = millis();
        my_state.firmware_version = this->firmware_version_;
        memcpy(my_state.src_addr, this->mac_address_bytes_, 6);
        my_state.rssi = 0;  // Not applicable for self
        my_state.uptime = 0;  // Will be updated in update_system_info_()
        my_state.voltage = 0.0f;
        my_state.current = 0.0f;
        my_state.power = 0.0f;
        
        ESP_LOGI(TAG, "  Device role: %s (index: %d)", tlv_device_role_to_string(this->device_role_), this->device_role_);
        ESP_LOGI(TAG, "  Initialized self in device_states_[%d]", this->device_role_);
    } else {
        ESP_LOGW(TAG, "  ⚠️ Device role %d is invalid, not initializing device_states_ entry", this->device_role_);
    }
}

void PowerSyncComponent::update_system_info_()
{
    // Update uptime (in seconds)
    this->uptime_seconds_ = millis() / 1000;

    // Update free memory
    this->free_memory_ = esp_get_free_heap_size();
    
    // Update current device's uptime in device_states_ array
    if (this->device_role_ < MAX_DEVICE_ROLES) {
        DeviceState& my_state = this->device_states_[this->device_role_];
        if (my_state.is_valid && my_state.role == this->device_role_) {
            my_state.uptime = this->uptime_seconds_;
            my_state.last_update_time = millis();
        }
    }
}

std::vector<uint8_t> PowerSyncComponent::build_tlv_packet_()
{
    std::vector<uint8_t> payload;

    ESP_LOGD(TAG, "Building composite TLV data packet containing all TLV types");

    // Add all TLV types (matching the YAML configuration order)
    this->add_tlv_uptime_(payload);
    this->add_tlv_device_id_(payload);
    this->add_tlv_mac_address_(payload);
    this->add_tlv_compile_time_(payload);
    this->add_tlv_firmware_version_(payload);
    this->add_tlv_ac_voltage_(payload);
    this->add_tlv_ac_current_(payload);
    this->add_tlv_ac_frequency_(payload);
    this->add_tlv_ac_power_(payload);
    this->add_tlv_device_role_(payload);
    this->add_tlv_free_memory_(payload);
    this->add_tlv_status_flags_(payload);

    ESP_LOGD(TAG, "Composite TLV data packet construction completed:");
    ESP_LOGD(TAG, "- Total packet length: %d bytes", payload.size());
    ESP_LOGD(TAG, "- Number of TLV types included: %d", 12);

    return payload;
}

void PowerSyncComponent::broadcast_tlv_data_()
{
    ESP_LOGD(TAG, "📡 ESP-NOW ready, sending TLV broadcast packet...");

    std::vector<uint8_t> payload = this->build_tlv_packet_();

    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcast_addr, payload.data(), payload.size());

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW broadcast failed: %s", esp_err_to_name(result));
    }
}

void PowerSyncComponent::handle_packet_received_(const uint8_t *src_addr, const uint8_t *data, int len, int rssi)
{
    ESP_LOGD(TAG, "Received ESP-NOW broadcast packet:");
    ESP_LOGD(TAG, "- Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             src_addr[0], src_addr[1], src_addr[2], src_addr[3], src_addr[4], src_addr[5]);
    ESP_LOGD(TAG, "- Data length: %d bytes", len);
    ESP_LOGD(TAG, "- RSSI: %d dBm", rssi);

    // Log first few bytes of data for debugging
    if (len > 0) {
        std::string hex_data;
        for (int i = 0; i < std::min(len, 16); i++) {
            char hex_byte[4];
            snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
            hex_data += hex_byte;
        }
        ESP_LOGV(TAG, "- Data content (first 16 bytes): %s", hex_data.c_str());
    }

    // Send packet data to message queue
    if (this->message_queue_ != nullptr && len > 0) {
        PowerSyncMessage msg;
        msg.type = DATA_TLV_RECEIVED;
        msg.rssi = rssi;
        
        // Copy source MAC address
        memcpy(msg.src_addr, src_addr, 6);
        
        // Copy packet data (limit to maximum message body size)
        msg.body_length = std::min(static_cast<size_t>(len), MAX_MESSAGE_BODY_SIZE);
        memcpy(msg.body, data, msg.body_length);
        
        // Send to queue (non-blocking)
        BaseType_t result = xQueueSend(this->message_queue_, &msg, 0);
        if (result != pdPASS) {
            ESP_LOGW(TAG, "Failed to send received packet to message queue (queue full)");
        } else {
            ESP_LOGD(TAG, "Packet sent to message queue successfully");
        }
    }

    ESP_LOGD(TAG, "📦 Trigger packet reception notification effect");
    this->trigger_packet_received_effect_();
}

void PowerSyncComponent::handle_unknown_peer_(const uint8_t *src_addr, const uint8_t *data, int len, int rssi)
{
    ESP_LOGI(TAG, "🚨 Unknown ESP-NOW peer detected!");
    ESP_LOGI(TAG, "Unknown peer information:");
    ESP_LOGI(TAG, "- Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             src_addr[0], src_addr[1], src_addr[2], src_addr[3], src_addr[4], src_addr[5]);
    ESP_LOGI(TAG, "- Data length: %d bytes", len);
    ESP_LOGI(TAG, "- RSSI: %d dBm", rssi);

    ESP_LOGI(TAG, "🔴 Trigger red LED warning effect");
    this->trigger_alert_red_effect_();
}

void PowerSyncComponent::trigger_packet_received_effect_()
{
    // RGB LED effects have been removed
    // This method is kept as a stub for backward compatibility
    ESP_LOGD(TAG, "Packet received effect triggered (RGB removed)");
}

void PowerSyncComponent::trigger_alert_red_effect_()
{
    // RGB LED effects have been removed
    // This method is kept as a stub for backward compatibility
    ESP_LOGD(TAG, "Alert red effect triggered (RGB removed)");
}

// TLV helper methods implementation
void PowerSyncComponent::add_tlv_uptime_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding UPTIME TLV");
    this->add_tlv_uint32_(payload, TLV_TYPE_UPTIME, this->uptime_seconds_);
    ESP_LOGD(TAG, "- Uptime: %lu seconds", this->uptime_seconds_);
}

void PowerSyncComponent::add_tlv_device_id_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding DEVICE_ID TLV");
    this->add_tlv_string_(payload, TLV_TYPE_DEVICE_ID, this->device_id_);
    ESP_LOGD(TAG, "- Device ID: %s", this->device_id_.c_str());
}

void PowerSyncComponent::add_tlv_mac_address_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding MAC_ADDRESS TLV");
    this->add_tlv_bytes_(payload, TLV_TYPE_MAC_ADDRESS, this->mac_address_bytes_, 6);
    ESP_LOGD(TAG, "- MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
             this->mac_address_bytes_[0], this->mac_address_bytes_[1], this->mac_address_bytes_[2],
             this->mac_address_bytes_[3], this->mac_address_bytes_[4], this->mac_address_bytes_[5]);
}

void PowerSyncComponent::add_tlv_compile_time_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding COMPILE_TIME TLV");
    this->add_tlv_string_(payload, TLV_TYPE_COMPILE_TIME, this->compile_time_);
    ESP_LOGD(TAG, "- Compile time: %s", this->compile_time_.c_str());
}

void PowerSyncComponent::add_tlv_firmware_version_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding FIRMWARE_VER TLV");
    this->add_tlv_string_(payload, TLV_TYPE_FIRMWARE_VER, this->firmware_version_);
    ESP_LOGD(TAG, "- Firmware version: %s", this->firmware_version_.c_str());
}

void PowerSyncComponent::add_tlv_ac_voltage_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding AC_VOLTAGE TLV (from global variable)");
    this->add_tlv_float_(payload, TLV_TYPE_AC_VOLTAGE, this->tlv_ac_voltage_);
    ESP_LOGD(TAG, "- AC voltage: %.1f V", this->tlv_ac_voltage_);
}

void PowerSyncComponent::add_tlv_ac_current_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding AC_CURRENT TLV (fixed-point version)");
    this->add_tlv_int32_(payload, TLV_TYPE_AC_CURRENT, this->tlv_ac_current_ma_);
    ESP_LOGD(TAG, "- AC current: %d mA (%.3f A)", this->tlv_ac_current_ma_, this->tlv_ac_current_ma_ / 1000.0f);
}

void PowerSyncComponent::add_tlv_ac_frequency_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding AC_FREQUENCY TLV (from global variable)");
    this->add_tlv_float_(payload, TLV_TYPE_AC_FREQUENCY, this->tlv_ac_frequency_);
    ESP_LOGD(TAG, "- AC frequency: %.2f Hz", this->tlv_ac_frequency_);
}

void PowerSyncComponent::add_tlv_ac_power_(std::vector<uint8_t> &payload)
{
    // we use INFO level here because power is a key metric
    ESP_LOGD(TAG, "Adding AC_POWER TLV (fixed-point version)");
    this->add_tlv_int32_(payload, TLV_TYPE_AC_POWER, this->tlv_ac_power_mw_);
    ESP_LOGD(TAG, "- AC power: %d mW (%.3f W)", this->tlv_ac_power_mw_, this->tlv_ac_power_mw_ / 1000.0f);
}

void PowerSyncComponent::add_tlv_device_role_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding DEVICE_ROLE TLV");
    payload.push_back(TLV_TYPE_DEVICE_ROLE);
    payload.push_back(1); // Length: 1 byte
    uint8_t role_value = static_cast<uint8_t>(this->device_role_);
    payload.push_back(role_value);
    
    // Use shared TLV helper function from powersync_tlv_format.h (C-compatible)
    ESP_LOGD(TAG, "- Device role: %s (%d)", 
             tlv_device_role_to_string(role_value), 
             role_value);
}

void PowerSyncComponent::add_tlv_free_memory_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding FREE_MEMORY TLV");
    this->add_tlv_uint32_(payload, TLV_TYPE_FREE_MEMORY, this->free_memory_);
    ESP_LOGD(TAG, "- Free memory: %lu Bytes (%.1f KB)", this->free_memory_, this->free_memory_ / 1024.0f);
}

void PowerSyncComponent::add_tlv_status_flags_(std::vector<uint8_t> &payload)
{
    ESP_LOGD(TAG, "Adding STATUS_FLAGS TLV");
    uint16_t status_flags = STATUS_POWER_ON | STATUS_ESP_NOW_ACTIVE;
    payload.push_back(TLV_TYPE_STATUS_FLAGS);
    payload.push_back(2); // Length
    payload.push_back((status_flags >> 8) & 0xFF);
    payload.push_back(status_flags & 0xFF);
    ESP_LOGD(TAG, "- Status flags: 0x%04X", status_flags);
}

// Utility methods implementation
void PowerSyncComponent::add_tlv_uint32_(std::vector<uint8_t> &payload, uint8_t type, uint32_t value)
{
    payload.push_back(type);
    payload.push_back(4); // Length
    payload.push_back((value >> 24) & 0xFF);
    payload.push_back((value >> 16) & 0xFF);
    payload.push_back((value >> 8) & 0xFF);
    payload.push_back(value & 0xFF);
}

void PowerSyncComponent::add_tlv_int32_(std::vector<uint8_t> &payload, uint8_t type, int32_t value)
{
    payload.push_back(type);
    payload.push_back(4); // Length
    payload.push_back((value >> 24) & 0xFF);
    payload.push_back((value >> 16) & 0xFF);
    payload.push_back((value >> 8) & 0xFF);
    payload.push_back(value & 0xFF);
}

void PowerSyncComponent::add_tlv_float_(std::vector<uint8_t> &payload, uint8_t type, float value)
{
    uint32_t float_bits;
    memcpy(&float_bits, &value, sizeof(float));
    this->add_tlv_uint32_(payload, type, float_bits);
}

void PowerSyncComponent::add_tlv_string_(std::vector<uint8_t> &payload, uint8_t type, const std::string &value)
{
    payload.push_back(type);
    payload.push_back(value.length());
    for (char c : value) {
        payload.push_back(c);
    }
}

void PowerSyncComponent::add_tlv_bytes_(std::vector<uint8_t> &payload, uint8_t type, const uint8_t *data, uint8_t len)
{
    payload.push_back(type);
    payload.push_back(len);
    for (uint8_t i = 0; i < len; i++) {
        payload.push_back(data[i]);
    }
}

// Public API methods
void PowerSyncComponent::increment_button_press_count()
{
    this->press_count_++;
    ESP_LOGI(TAG, "Button press count incremented to: %lu", this->press_count_);
}

void PowerSyncComponent::set_ac_voltage(float voltage)
{
    // Update TLV transmission variable
    this->tlv_ac_voltage_ = voltage;
    
    // Update device_states_ array for current device
    if (this->device_role_ < MAX_DEVICE_ROLES) {
        DeviceState& my_state = this->device_states_[this->device_role_];
        my_state.voltage = voltage;
        my_state.last_update_time = millis();
        my_state.is_valid = true;
        my_state.role = this->device_role_;
    }
    
    // Notify ESP-NOW task of data change
    this->send_data_changed_notification_();
}

void PowerSyncComponent::set_ac_current(float current)
{
    // Update TLV transmission variable (convert A to mA)
    this->tlv_ac_current_ma_ = static_cast<int32_t>(current * 1000.0f);
    
    // Update device_states_ array for current device
    if (this->device_role_ < MAX_DEVICE_ROLES) {
        DeviceState& my_state = this->device_states_[this->device_role_];
        my_state.current = current;
        my_state.last_update_time = millis();
        my_state.is_valid = true;
        my_state.role = this->device_role_;
    }
    
    // Notify ESP-NOW task of data change
    this->send_data_changed_notification_();
}

void PowerSyncComponent::set_ac_frequency(float frequency)
{
    // Update TLV transmission variable
    this->tlv_ac_frequency_ = frequency;
    
    // Note: frequency is not stored in DeviceState, only used for TLV broadcast
}

void PowerSyncComponent::set_ac_power(float power)
{
    // Update TLV transmission variable (convert W to mW)
    this->tlv_ac_power_mw_ = static_cast<int32_t>(power * 1000.0f);
    
    // Update device_states_ array for current device
    if (this->device_role_ < MAX_DEVICE_ROLES) {
        DeviceState& my_state = this->device_states_[this->device_role_];
        my_state.power = power;
        my_state.last_update_time = millis();
        my_state.is_valid = true;
        my_state.role = this->device_role_;
    }
    
    // Notify ESP-NOW task of data change (power is critical for immediate broadcast)
    this->send_data_changed_notification_();
}

void PowerSyncComponent::trigger_broadcast()
{
    if (false == this->espnow_ready_) {
        return;
    }
    this->broadcast_tlv_data_();
}

void PowerSyncComponent::send_tlv_command()
{
    if (this->message_queue_ != nullptr) {
        PowerSyncMessage msg;
        msg.type = CMD_SEND_TLV;
        msg.body_length = 0;  // No body needed for command messages
        msg.rssi = 0;         // Not applicable for command messages
        memset(msg.src_addr, 0, 6);  // Not applicable for command messages
        
        BaseType_t result = xQueueSend(this->message_queue_, &msg, pdMS_TO_TICKS(100));
        if (result != pdPASS) {
            ESP_LOGW(TAG, "Failed to send CMD_SEND_TLV to message queue");
        } else {
            ESP_LOGD(TAG, "CMD_SEND_TLV sent to message queue successfully");
        }
    } else {
        ESP_LOGE(TAG, "Message queue not initialized");
    }
}

void PowerSyncComponent::send_data_changed_notification_()
{
    if (this->message_queue_ != nullptr) {
        PowerSyncMessage msg;
        msg.type = CMD_DATA_CHANGED;
        msg.body_length = 0;  // No body needed for command messages
        msg.rssi = 0;         // Not applicable for command messages
        memset(msg.src_addr, 0, 6);  // Not applicable for command messages
        
        // Send with minimal timeout (non-blocking)
        BaseType_t result = xQueueSend(this->message_queue_, &msg, 0);
        if (result != pdPASS) {
            ESP_LOGV(TAG, "Failed to send CMD_DATA_CHANGED to message queue (queue full)");
        } else {
            ESP_LOGV(TAG, "CMD_DATA_CHANGED sent to message queue successfully");
        }
    }
}

// TLV parsing callback implementation
bool PowerSyncComponent::tlv_parse_callback_(uint8_t type, uint8_t length, const uint8_t* value, void* user_data)
{
    // user_data is the TLVParseContext pointer
    TLVParseContext* ctx = static_cast<TLVParseContext*>(user_data);
    
    // Use TLV helper function to get type name
    const char* type_name = tlv_type_to_string(type);
    
    ESP_LOGD(TAG, "📦 TLV Entry: Type=0x%02X (%s), Length=%d", type, type_name, length);
    
    // Parse different TLV types and store in context
    switch (type) {
        case TLV_TYPE_UPTIME: {
            if (length == TLV_SIZE_UPTIME) {
                ctx->device_state.uptime = TLV_UINT32_FROM_BE(value);
                ctx->has_uptime = true;
                ESP_LOGD(TAG, "   ⏱️  Uptime: %lu seconds (%.2f hours)", 
                         ctx->device_state.uptime, ctx->device_state.uptime / 3600.0f);
            } else {
                ESP_LOGW(TAG, "   ⚠️  Invalid uptime length: %d (expected %d)", length, TLV_SIZE_UPTIME);
            }
            break;
        }
        
        case TLV_TYPE_TIMESTAMP: {
            if (length == TLV_SIZE_TIMESTAMP) {
                uint32_t timestamp = TLV_UINT32_FROM_BE(value);
                ESP_LOGD(TAG, "   🕐 Timestamp: %lu", timestamp);
            }
            break;
        }
        
        case TLV_TYPE_DEVICE_ID: {
            if (length > 0 && length <= TLV_MAX_DEVICE_ID_LEN) {
                char device_id[TLV_MAX_DEVICE_ID_LEN + 1] = {0};
                memcpy(device_id, value, length);
                ESP_LOGD(TAG, "   🏷️  Device ID: %s", device_id);
            }
            break;
        }
        
        case TLV_TYPE_FIRMWARE_VER: {
            if (length > 0 && length <= TLV_MAX_FIRMWARE_VER_LEN) {
                char firmware[TLV_MAX_FIRMWARE_VER_LEN + 1] = {0};
                memcpy(firmware, value, length);
                ctx->device_state.firmware_version = std::string(firmware);
                ctx->has_firmware = true;
                ESP_LOGD(TAG, "   📌 Firmware: %s", firmware);
            }
            break;
        }
        
        case TLV_TYPE_MAC_ADDRESS: {
            if (length == TLV_SIZE_MAC_ADDRESS) {
                ESP_LOGD(TAG, "   🌐 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                        value[0], value[1], value[2], value[3], value[4], value[5]);
            }
            break;
        }
        
        case TLV_TYPE_COMPILE_TIME: {
            if (length > 0 && length <= TLV_MAX_COMPILE_TIME_LEN) {
                char compile_time[TLV_MAX_COMPILE_TIME_LEN + 1] = {0};
                memcpy(compile_time, value, length);
                ESP_LOGD(TAG, "   🔨 Compile time: %s", compile_time);
            }
            break;
        }
        
        case TLV_TYPE_FREE_MEMORY: {
            if (length == TLV_SIZE_FREE_MEMORY) {
                uint32_t free_mem = TLV_UINT32_FROM_BE(value);
                ESP_LOGD(TAG, "   💾 Free memory: %lu bytes (%.1f KB)", free_mem, free_mem / 1024.0f);
            }
            break;
        }
        
        case TLV_TYPE_DEVICE_ROLE: {
            if (length == TLV_SIZE_DEVICE_ROLE) {
                uint8_t role = value[0];
                ctx->device_state.role = static_cast<DeviceRole>(role);
                ctx->has_role = true;
                const char* role_name = tlv_device_role_to_string(role);
                ESP_LOGD(TAG, "   🎭 Device role: %s (%d)", role_name, role);
            }
            break;
        }
        
        case TLV_TYPE_AC_VOLTAGE: {
            if (length == TLV_SIZE_AC_VOLTAGE) {
                TLV_FLOAT32_FROM_BE(value, ctx->device_state.voltage);
                ctx->has_voltage = true;
                ESP_LOGD(TAG, "   ⚡ AC Voltage: %.1f V", ctx->device_state.voltage);
            }
            break;
        }
        
        case TLV_TYPE_AC_CURRENT: {
            if (length == TLV_SIZE_AC_CURRENT) {
                int32_t current_ma = TLV_INT32_FROM_BE(value);
                ctx->device_state.current = TLV_CURRENT_MA_TO_A(current_ma);
                ctx->has_current = true;
                ESP_LOGD(TAG, "   🔌 AC Current: %.3f A (%d mA)", ctx->device_state.current, current_ma);
            }
            break;
        }
        
        case TLV_TYPE_AC_FREQUENCY: {
            if (length == TLV_SIZE_AC_FREQUENCY) {
                float frequency;
                TLV_FLOAT32_FROM_BE(value, frequency);
                ESP_LOGD(TAG, "   📻 AC Frequency: %.2f Hz", frequency);
            }
            break;
        }
        
        case TLV_TYPE_AC_POWER: {
            if (length == TLV_SIZE_AC_POWER) {
                int32_t power_mw = TLV_INT32_FROM_BE(value);
                ctx->device_state.power = TLV_POWER_MW_TO_W(power_mw);
                ctx->has_power = true;
                ESP_LOGD(TAG, "   💡 AC Power: %.3f W (%d mW)", ctx->device_state.power, power_mw);
            }
            break;
        }
        
        case TLV_TYPE_AC_POWER_FACTOR: {
            if (length == TLV_SIZE_AC_POWER_FACTOR) {
                float power_factor;
                TLV_FLOAT32_FROM_BE(value, power_factor);
                ESP_LOGD(TAG, "   📊 Power Factor: %.3f", power_factor);
            }
            break;
        }
        
        case TLV_TYPE_ENERGY_TOTAL: {
            if (length == TLV_SIZE_ENERGY_TOTAL) {
                float energy;
                TLV_FLOAT32_FROM_BE(value, energy);
                ESP_LOGD(TAG, "   🔋 Total Energy: %.2f kWh", energy);
            }
            break;
        }
        
        case TLV_TYPE_ENERGY_TODAY: {
            if (length == TLV_SIZE_ENERGY_TODAY) {
                float energy;
                TLV_FLOAT32_FROM_BE(value, energy);
                ESP_LOGD(TAG, "   📅 Today's Energy: %.2f kWh", energy);
            }
            break;
        }
        
        case TLV_TYPE_STATUS_FLAGS: {
            if (length == TLV_SIZE_STATUS_FLAGS) {
                uint16_t flags = TLV_UINT16_FROM_BE(value);
                ESP_LOGD(TAG, "   🚦 Status Flags: 0x%04X", flags);
                if (flags & STATUS_FLAG_POWER_ON) ESP_LOGD(TAG, "      - Power ON");
                if (flags & STATUS_FLAG_CALIBRATING) ESP_LOGD(TAG, "      - Calibrating");
                if (flags & STATUS_FLAG_ERROR) ESP_LOGD(TAG, "      - Error");
                if (flags & STATUS_FLAG_LOW_BATTERY) ESP_LOGD(TAG, "      - Low Battery");
                if (flags & STATUS_FLAG_WIFI_CONNECTED) ESP_LOGD(TAG, "      - WiFi Connected");
                if (flags & STATUS_FLAG_ESP_NOW_ACTIVE) ESP_LOGD(TAG, "      - ESP-NOW Active");
            }
            break;
        }
        
        case TLV_TYPE_ERROR_CODE: {
            if (length == TLV_SIZE_ERROR_CODE) {
                uint16_t error_code = TLV_UINT16_FROM_BE(value);
                ESP_LOGD(TAG, "   ❌ Error Code: 0x%04X", error_code);
            }
            break;
        }
        
        case TLV_TYPE_TEMPERATURE: {
            if (length == TLV_SIZE_TEMPERATURE) {
                float temperature;
                TLV_FLOAT32_FROM_BE(value, temperature);
                ESP_LOGD(TAG, "   🌡️  Temperature: %.1f °C", temperature);
            }
            break;
        }
        
        case TLV_TYPE_HUMIDITY: {
            if (length == TLV_SIZE_HUMIDITY) {
                float humidity;
                TLV_FLOAT32_FROM_BE(value, humidity);
                ESP_LOGD(TAG, "   💧 Humidity: %.1f %%", humidity);
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "   ⚠️  Unknown TLV type: 0x%02X", type);
            break;
    }
    
    return true; // Continue parsing next TLV entry
}

// FreeRTOS task function implementation
void PowerSyncComponent::espnow_task_function_(void *pvParameters)
{
    PowerSyncComponent *component = static_cast<PowerSyncComponent *>(pvParameters);
    
    ESP_LOGI(TAG, "ESP-NOW task started");
    
    uint32_t last_broadcast_time = 0;
    uint32_t last_system_update_time = 0;
    
    while (true) {
        uint32_t now = millis();
        
        // Process messages from the queue
        PowerSyncMessage msg;
        BaseType_t result = xQueueReceive(component->message_queue_, &msg, pdMS_TO_TICKS(10));
        if (result == pdPASS) {
            switch (msg.type) {
                case CMD_SEND_TLV:
                    ESP_LOGI(TAG, "Processing CMD_SEND_TLV message from queue");
                    component->broadcast_tlv_data_();
                    break;
                    
                case CMD_DATA_CHANGED: {
                    ESP_LOGV(TAG, "Processing CMD_DATA_CHANGED message from queue");
                    
                    // Get current power value in watts
                    float current_power_w = component->tlv_ac_power_mw_ / 1000.0f;
                    
                    // Calculate absolute power change
                    float power_change_w = std::abs(current_power_w - component->last_broadcast_power_w_);
                    
                    ESP_LOGV(TAG, "Power change check: current=%.3f W, last_broadcast=%.3f W, change=%.3f W, threshold=%.1f W",
                             current_power_w, component->last_broadcast_power_w_, 
                             power_change_w, component->power_change_threshold_w_);
                    
                    // If power change exceeds threshold, trigger immediate broadcast
                    if (power_change_w >= component->power_change_threshold_w_) {
                        ESP_LOGI(TAG, "⚡ Power change (%.1f W) exceeds threshold (%.1f W) - Triggering immediate broadcast",
                                 power_change_w, component->power_change_threshold_w_);
                        
                        component->broadcast_tlv_data_();
                        
                        // Update last broadcast power value
                        component->last_broadcast_power_w_ = current_power_w;
                        
                        // Reset last_broadcast_time to prevent double broadcast
                        last_broadcast_time = now;
                    } else {
                        ESP_LOGV(TAG, "Power change (%.1f W) within threshold (%.1f W) - No immediate broadcast needed",
                                 power_change_w, component->power_change_threshold_w_);
                    }
                    break;
                }
                    
                case DATA_TLV_RECEIVED:
                    ESP_LOGD(TAG, "Processing DATA_TLV_RECEIVED message from queue");
                    ESP_LOGD(TAG, "- Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                             msg.src_addr[0], msg.src_addr[1], msg.src_addr[2], 
                             msg.src_addr[3], msg.src_addr[4], msg.src_addr[5]);
                    ESP_LOGD(TAG, "- Data length: %zu bytes", msg.body_length);
                    ESP_LOGD(TAG, "- RSSI: %d dBm", msg.rssi);
                    
                    // Validate TLV packet structure before parsing
                    if (tlv_validate_packet(msg.body, msg.body_length)) {
                        ESP_LOGD(TAG, "✅ TLV packet structure is valid");
                        
                        // Count TLV entries
                        int entry_count = tlv_count_entries(msg.body, msg.body_length);
                        ESP_LOGD(TAG, "📊 TLV packet contains %d entries", entry_count);
                        
                        // Create parsing context
                        TLVParseContext ctx;
                        ctx.device_state.rssi = msg.rssi;
                        memcpy(ctx.device_state.src_addr, msg.src_addr, 6);
                        ctx.device_state.last_update_time = millis();
                        
                        // Parse TLV data using streaming parser
                        int parsed = tlv_parse_stream(msg.body, msg.body_length, 
                                                     component->tlv_parse_callback_, 
                                                     &ctx);
                        
                        if (parsed >= 0) {
                            ESP_LOGD(TAG, "✅ Successfully parsed %d TLV entries", parsed);
                            
                            // Update device state if we have valid role and at least one measurement
                            if (ctx.has_role && (ctx.has_voltage || ctx.has_current || ctx.has_power)) {
                                // Check if received role matches current device's role (loopback detection)
                                if (ctx.device_state.role == component->device_role_) {
                                    ESP_LOGE(TAG, "❌ Received packet with same role as current device (%s) - Possible loopback! Rejecting update.",
                                             tlv_device_role_to_string(ctx.device_state.role));
                                    ESP_LOGE(TAG, "   Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                                             msg.src_addr[0], msg.src_addr[1], msg.src_addr[2], 
                                             msg.src_addr[3], msg.src_addr[4], msg.src_addr[5]);
                                    ESP_LOGE(TAG, "   Self MAC:   %02X:%02X:%02X:%02X:%02X:%02X",
                                             component->mac_address_bytes_[0], component->mac_address_bytes_[1], 
                                             component->mac_address_bytes_[2], component->mac_address_bytes_[3],
                                             component->mac_address_bytes_[4], component->mac_address_bytes_[5]);
                                    break;  // Skip update for same role
                                }
                                
                                // Get reference to device state for this role
                                DeviceState& dev_state = component->device_states_[ctx.device_state.role];
                                
                                // Always update role, RSSI, MAC, and timestamp
                                dev_state.role = ctx.device_state.role;
                                dev_state.rssi = msg.rssi;
                                memcpy(dev_state.src_addr, msg.src_addr, 6);
                                dev_state.last_update_time = millis();
                                
                                // Only copy fields that were actually parsed (has_* flags)
                                if (ctx.has_voltage) {
                                    dev_state.voltage = ctx.device_state.voltage;
                                }
                                if (ctx.has_current) {
                                    dev_state.current = ctx.device_state.current;
                                }
                                if (ctx.has_power) {
                                    dev_state.power = ctx.device_state.power;
                                }
                                if (ctx.has_uptime) {
                                    dev_state.uptime = ctx.device_state.uptime;
                                }
                                if (ctx.has_firmware) {
                                    dev_state.firmware_version = ctx.device_state.firmware_version;
                                }
                                
                                // Mark device state as valid
                                dev_state.is_valid = true;
                                
                                ESP_LOGD(TAG, "💾 Updated device state for role %s (V:%d I:%d P:%d U:%d F:%d)", 
                                         tlv_device_role_to_string(ctx.device_state.role),
                                         ctx.has_voltage, ctx.has_current, ctx.has_power,
                                         ctx.has_uptime, ctx.has_firmware);
                                
                                // Log updated values for debugging
                                component->update_device_state_(ctx.device_state.role, msg.src_addr, msg.rssi);
                            } else {
                                ESP_LOGW(TAG, "⚠️ Missing required fields (role:%d, voltage:%d, current:%d, power:%d)",
                                         ctx.has_role, ctx.has_voltage, ctx.has_current, ctx.has_power);
                            }
                        } else {
                            switch (parsed) {
                                case -1:
                                    ESP_LOGE(TAG, "❌ TLV parse error: Invalid parameters");
                                    break;
                                case -2:
                                    ESP_LOGE(TAG, "❌ TLV parse error: Malformed packet");
                                    break;
                                case -3:
                                    ESP_LOGW(TAG, "⚠️ TLV parsing terminated early by callback");
                                    break;
                                default:
                                    ESP_LOGE(TAG, "❌ TLV parse error: Unknown error code %d", parsed);
                                    break;
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "❌ Invalid TLV packet structure, skipping parsing");
                    }
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown message type received: %d", msg.type);
                    break;
            }
        }
        
        // Update data age for all valid devices before making power decisions
        component->update_all_device_data_age_();
        
        // Make power management decisions every loop iteration (every 10ms due to queue timeout)
        component->make_power_management_decisions_();
        
        // Update system info at specified interval
        if (now - last_system_update_time >= component->system_update_interval_) {
            component->update_system_info_();
            last_system_update_time = now;
        }
        
        // Broadcast TLV data at specified interval
        if (component->espnow_ready_ && now - last_broadcast_time >= component->broadcast_interval_) {
            component->broadcast_tlv_data_();
            last_broadcast_time = now;
            
            // Update last broadcast power value for change detection
            component->last_broadcast_power_w_ = component->tlv_ac_power_mw_ / 1000.0f;
        }
        
        // Dump device states table every 5 seconds
        static uint32_t last_dump_time = 0;
        if (now - last_dump_time >= 5000) {  // 5 seconds interval
            component->dump_device_states_table_();
            last_dump_time = now;
        }
        
        // Note: We don't need additional delay here since xQueueReceive already provides a 10ms timeout
    }
}

// Device state management methods implementation

const DeviceState* PowerSyncComponent::get_device_state(DeviceRole role) const
{
    if (role >= MAX_DEVICE_ROLES) {
        return nullptr;
    }
    
    const DeviceState& state = this->device_states_[role];
    if (!state.is_valid) {
        return nullptr;
    }
    
    // Check if device data is still fresh (within timeout period)
    uint32_t now = millis();
    if (now - state.last_update_time > DEVICE_STATE_TIMEOUT) {
        return nullptr; // Data too old
    }
    
    return &state;
}

bool PowerSyncComponent::is_device_active(DeviceRole role) const
{
    return get_device_state(role) != nullptr;
}

void PowerSyncComponent::clear_inactive_devices()
{
    uint32_t now = millis();
    int cleared_count = 0;
    
    for (size_t i = 0; i < MAX_DEVICE_ROLES; i++) {
        DeviceState& state = this->device_states_[i];
        if (state.is_valid && (now - state.last_update_time > DEVICE_STATE_TIMEOUT)) {
            ESP_LOGI(TAG, "⏰ Clearing inactive device: role %s (last update: %lu ms ago)",
                     tlv_device_role_to_string(i), now - state.last_update_time);
            state.is_valid = false;
            cleared_count++;
        }
    }
    
    if (cleared_count > 0) {
        ESP_LOGI(TAG, "🧹 Cleared %d inactive device(s)", cleared_count);
    }
}

void PowerSyncComponent::update_device_state_(DeviceRole role, const uint8_t *src_addr, int rssi)
{
    if (role >= MAX_DEVICE_ROLES) {
        ESP_LOGW(TAG, "⚠️ Invalid device role: %d", role);
        return;
    }
    
    const DeviceState& state = this->device_states_[role];
    
    // Log device state update (only show what was actually received)
    ESP_LOGI(TAG, "📝 Device state summary:");
    ESP_LOGI(TAG, "   Role: %s", tlv_device_role_to_string(role));
    ESP_LOGI(TAG, "   MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             src_addr[0], src_addr[1], src_addr[2], src_addr[3], src_addr[4], src_addr[5]);
    
    // Only log non-zero values (assuming 0 means not set)
    if (state.voltage != 0.0f) {
        ESP_LOGI(TAG, "   Voltage: %.1f V", state.voltage);
    }
    if (state.current != 0.0f) {
        ESP_LOGI(TAG, "   Current: %.3f A", state.current);
    }
    if (state.power != 0.0f) {
        ESP_LOGI(TAG, "   Power: %.3f W", state.power);
    }
    ESP_LOGI(TAG, "   RSSI: %d dBm", rssi);
    if (state.uptime > 0) {
        ESP_LOGI(TAG, "   Uptime: %lu s (%.2f hours)", state.uptime, state.uptime / 3600.0f);
    }
    if (!state.firmware_version.empty()) {
        ESP_LOGI(TAG, "   Firmware: %s", state.firmware_version.c_str());
    }
}

void PowerSyncComponent::update_all_device_data_age_()
{
    // Update data age for all valid device states
    uint32_t now = millis();
    int updated_count = 0;
    
    for (size_t i = 0; i < MAX_DEVICE_ROLES; i++) {
        DeviceState& state = this->device_states_[i];
        
        // Only update age for valid devices
        if (state.is_valid) {
            // Calculate data age in milliseconds
            state.data_age_ms = now - state.last_update_time;
            updated_count++;
        }
    }
    
    ESP_LOGV(TAG, "Updated data age for %d valid device(s)", updated_count);
}

void PowerSyncComponent::make_power_management_decisions_()
{
    // This function dispatches to role-specific strategy functions based on:
    // 1. Current device's role (this->device_role_)
    // 2. Network-wide device states (this->device_states_[])
    // 3. Own device state (this->device_states_[this->device_role_])
    
    // Dispatch to role-specific strategy function
    switch (this->device_role_) {
        case ROLE_GRID_INPUT:
            this->strategy_grid_input_();
            break;
            
        case ROLE_INVERTER_AC_INPUT:
            this->strategy_inverter_ac_input_();
            break;
            
        case ROLE_INVERTER_AC_OUTPUT:
            this->strategy_inverter_ac_output_();
            break;
            
        case ROLE_INVERTER_DC_BATTERY:
            this->strategy_inverter_dc_battery_();
            break;
            
        case ROLE_INVERTER_DC_GRID_POWER:
            this->strategy_inverter_dc_grid_power_();
            break;
            
        case ROLE_SINKER_AC_HEATER:
            this->strategy_sinker_ac_heater_();
            break;
            
        case ROLE_SINKER_DC_HEATER:
            this->strategy_sinker_dc_heater_();
            break;
            
        case ROLE_SINKER_AC_VEHICLE_CHARGER:
            this->strategy_sinker_ac_vehicle_charger_();
            break;
            
        case ROLE_SINKER_DC_VEHICLE_CHARGER:
            this->strategy_sinker_dc_vehicle_charger_();
            break;
            
        case ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL:
            this->strategy_solar_inverter_output_total_();
            break;
            
        case ROLE_UNKNOWN:
        default:
            ESP_LOGV(TAG, "No strategy defined for role: %s", 
                     tlv_device_role_to_string(this->device_role_));
            break;
    }
    
    // Update state duration tracking for all device roles (after strategy execution)
    // This is a generic feature that tracks continuous state duration based on power sign
    if (this->state_duration_sensor_ != nullptr) {
        this->update_state_duration_();
    }
    
    ESP_LOGV(TAG, "Power management decision cycle completed for role: %s", 
             tlv_device_role_to_string(this->device_role_));
}

// ============================================================================
// Role-Specific Strategy Functions
// ============================================================================

void PowerSyncComponent::strategy_inverter_ac_input_()
{
    // Strategy: Monitor own power for reverse feed (grid feed)
    // Priority 1: Check own power first - if negative (reverse feed), immediately disconnect relay
    // Priority 2: If own power is positive or zero, then monitor solar inverter for coordination
    // 
    // Protection Features:
    // - Edge-triggered logging: Log only on state change (normal ↔ grid feed)
    // - Edge-triggered relay control: Send relay commands only on state transitions
    // - Rate-limited reminders: Periodic logs during persistent grid feed (every 5s)
    
    ESP_LOGV(TAG, "🔌 Executing INVERTER_AC_INPUT strategy");
    
    uint32_t now = millis();
    
    // ========================================================================
    // Priority 1: Check own device power for reverse feed (grid feed)
    // ========================================================================
    const DeviceState* own_state = this->get_device_state(this->device_role_);
    
    if (own_state != nullptr) {
        // Check if own power is negative (reverse power flow / grid feed)
        float own_power_w = own_state->power;
        
        if (own_power_w < 0.0f) {
            // Grid feed detected!
            
            // Edge-triggered logging AND relay control: Only act on state transition
            if (this->last_grid_feed_state_own_ != GRID_FEED_STATE_FEEDING) {
                ESP_LOGW(TAG, "⚠️ CRITICAL: Own power grid feed STARTED - Power: %.2f W", own_power_w);
                this->last_grid_feed_state_own_ = GRID_FEED_STATE_FEEDING;
                this->grid_feed_start_time_own_ = now;        // Record state start time
                this->last_grid_feed_log_time_own_ = now;     // Initialize last log time
                
                // Send relay trip command ONLY on state transition to FEEDING
                if (this->dlt645_relay_trip_sensor_ != nullptr) {
                    // CRITICAL: Use pulse mode (false -> true) to trigger on_state event
                    this->dlt645_relay_trip_sensor_->publish_state(false);
                    this->dlt645_relay_trip_sensor_->publish_state(true);
                    this->last_relay_trip_time_ = now;  // Track for logging purposes only
                    
                    ESP_LOGI(TAG, "✅ DLT645 relay trip pulse sent (own grid feed protection)");
                } else {
                    ESP_LOGE(TAG, "❌ DLT645 relay trip sensor not configured!");
                }
            }
            // Rate-limited reminder: Log periodically during persistent grid feed (no relay command)
            else if (now - this->last_grid_feed_log_time_own_ >= this->grid_feed_log_interval_) {
                // Calculate total duration since state started (from grid_feed_start_time_own_)
                uint32_t duration_s = (now - this->grid_feed_start_time_own_) / 1000;
                ESP_LOGW(TAG, "⚠️ Own power grid feed ONGOING - Power: %.2f W (已持续 %lu 秒)", 
                         own_power_w, duration_s);
                this->last_grid_feed_log_time_own_ = now;     // Update last log time for next rate limit check
                // NO relay command here - relay already tripped on state transition
            }
            // Return immediately - no need to check solar inverter data
            return;
            
        } else {
            // Power is positive or zero
            // no action needed. We may choose to close relay, but we will do it in the solar check below.
        }
    } else {
        ESP_LOGV(TAG, "⚠️ Own device state not available - cannot check for grid feed");
    }
    
    // ========================================================================
    // Priority 2: Check solar inverter output for reverse power
    // (Only executed if own power is not negative AND if enabled in configuration)
    // ========================================================================
    
    // Skip solar inverter check if disabled in YAML configuration
    if (!this->check_solar_inverter_power_) {
        ESP_LOGV(TAG, "ℹ️ Solar inverter power check disabled in configuration - skipping");
        return;
    }
    
    const DeviceState* solar_state = this->get_device_state(ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL);
    
    if (solar_state != nullptr) {
        // Check if data is fresh (within configured timeout)
        if (solar_state->data_age_ms <= this->power_decision_data_timeout_) {
            // Solar inverter is active and reporting with fresh data
            ESP_LOGV(TAG, "🌞 Solar inverter detected - Power: %.2f W (data age: %lu ms)", 
                     solar_state->power, solar_state->data_age_ms);
            
            // Check if solar power is negative (feeding to grid from solar inverter)
            // Note: Negative power means solar is feeding power back to grid
            //       Zero or positive power means no solar activity (night/cloudy) or consuming
            // To optimize the case that when the sun has just risen, the solar power may be still low
            // and unstable, we need to set a small threshold (configurable via solar_power_threshold) to avoid false triggering.
            if (solar_state->power < this->solar_power_threshold_w_) {
                // Solar grid feed detected! (power < 0 means feeding to grid)
                
                // Edge-triggered logging AND relay control: Only act on state transition
                if (this->last_grid_feed_state_solar_ != GRID_FEED_STATE_FEEDING) {
                    ESP_LOGW(TAG, "⚠️ CRITICAL: Solar inverter grid feed STARTED - Power: %.2f W (negative = feeding)", solar_state->power);
                    this->last_grid_feed_state_solar_ = GRID_FEED_STATE_FEEDING;
                    this->grid_feed_start_time_solar_ = now;      // Record state start time
                    this->last_grid_feed_log_time_solar_ = now;   // Initialize last log time
                    
                    // Send relay trip command ONLY on state transition to FEEDING
                    if (this->dlt645_relay_trip_sensor_ != nullptr) {
                        // CRITICAL: Use pulse mode (false -> true) to trigger on_state event
                        this->dlt645_relay_trip_sensor_->publish_state(false);
                        this->dlt645_relay_trip_sensor_->publish_state(true);
                        this->last_relay_trip_time_ = now;  // Track for logging purposes only
                        
                        ESP_LOGI(TAG, "✅ DLT645 relay trip pulse sent (solar grid feed protection)");
                    } else {
                        ESP_LOGE(TAG, "❌ DLT645 relay trip sensor not configured!");
                    }
                }
                // Rate-limited reminder: Log periodically during persistent grid feed (no relay command)
                else if (now - this->last_grid_feed_log_time_solar_ >= this->grid_feed_log_interval_) {
                    // Calculate total duration since state started (from grid_feed_start_time_solar_)
                    uint32_t duration_s = (now - this->grid_feed_start_time_solar_) / 1000;
                    ESP_LOGW(TAG, "⚠️ Solar inverter grid feed ONGOING - Power: %.2f W (已持续 %lu 秒)", 
                             solar_state->power, duration_s);
                    this->last_grid_feed_log_time_solar_ = now;   // Update last log time for next rate limit check
                    // NO relay command here - relay already tripped on state transition
                }
                
            } else {
                // Solar power is zero or positive (no solar activity or not feeding to grid)
                
                // Edge-triggered logging: Log state transition (grid feed/invalid → normal)
                if (this->last_grid_feed_state_solar_ == GRID_FEED_STATE_FEEDING) {
                    // Transition from GRID_FEED to NORMAL - relay was tripped, now restore
                    ESP_LOGI(TAG, "✅ Solar inverter grid feed RESOLVED - Power back to normal: %.2f W (no feeding)", solar_state->power);
                    this->last_grid_feed_state_solar_ = GRID_FEED_STATE_NORMAL;
                    
                    // Send relay close command (no rate limit on state change)
                    if (this->dlt645_relay_close_sensor_ != nullptr) {
                        // CRITICAL: Use pulse mode (false -> true) to trigger on_state event
                        this->dlt645_relay_close_sensor_->publish_state(false);
                        this->dlt645_relay_close_sensor_->publish_state(true);
                        this->last_relay_close_time_ = now;  // Track for logging purposes only
                        
                        ESP_LOGI(TAG, "✅ DLT645 relay close pulse sent (grid connection restored after grid feed resolved)");
                    } else {
                        ESP_LOGW(TAG, "⚠️ DLT645 relay close sensor not configured - cannot auto-restore grid connection");
                    }
                } else if (this->last_grid_feed_state_solar_ == GRID_FEED_STATE_INVALID) {
                    // First time detection with no solar activity - CRITICAL: Need to sync relay state
                    // We don't know the relay's actual state, so we proactively send close command
                    ESP_LOGI(TAG, "✅ Solar inverter state initialized - No solar activity: %.2f W", solar_state->power);
                    ESP_LOGI(TAG, "🔄 Syncing relay state on first detection (sending close command to ensure connection)");
                    this->last_grid_feed_state_solar_ = GRID_FEED_STATE_NORMAL;
                    
                    // Send relay close command (no rate limit on initial sync)
                    if (this->dlt645_relay_close_sensor_ != nullptr) {
                        // CRITICAL: Use pulse mode (false -> true) to trigger on_state event
                        this->dlt645_relay_close_sensor_->publish_state(false);
                        this->dlt645_relay_close_sensor_->publish_state(true);
                        this->last_relay_close_time_ = now;  // Track for logging purposes only
                        
                        ESP_LOGI(TAG, "✅ DLT645 relay close pulse sent (initial state sync)");
                    } else {
                        ESP_LOGW(TAG, "⚠️ DLT645 relay close sensor not configured - cannot sync relay state");
                    }
                } else if (this->last_grid_feed_state_solar_ == GRID_FEED_STATE_NORMAL) {
                    // Already in NORMAL state - no action needed (state-based optimization)
                    // Only send relay commands on state transitions to avoid unnecessary operations
                    ESP_LOGV(TAG, "ℹ️ Solar power remains normal: %.2f W (no relay action needed)", solar_state->power);
                }
                
                ESP_LOGV(TAG, "✅ Solar power is zero or positive (no solar feeding): %.2f W", solar_state->power);
            }
        } else {
            ESP_LOGW(TAG, "⚠️ Solar inverter data is stale (age: %lu ms, timeout: %lu ms) - Skipping power decision",
                     solar_state->data_age_ms, this->power_decision_data_timeout_);
            // we should trip the relay, in case we has totally lost power after inverter output
            if (this->last_grid_feed_state_solar_ == GRID_FEED_STATE_FEEDING) {
                ESP_LOGW(TAG, "⚠️ Solar inverter data stale during grid feed - Ensuring relay is tripped for safety");
                // Send relay trip command
                if (this->dlt645_relay_trip_sensor_ != nullptr) {
                    // CRITICAL: Use pulse mode (false -> true) to trigger on_state event
                    this->dlt645_relay_trip_sensor_->publish_state(false);
                    this->dlt645_relay_trip_sensor_->publish_state(true);
                    this->last_relay_trip_time_ = now;  // Track for logging purposes only
                    
                    ESP_LOGI(TAG, "✅ DLT645 relay trip pulse sent (solar data stale during grid feed)");
                } else {
                    ESP_LOGE(TAG, "❌ DLT645 relay trip sensor not configured!");
                }
            }
        }
    } else {
        ESP_LOGV(TAG, "ℹ️ Solar inverter data not available or expired");
    }
}

void PowerSyncComponent::strategy_grid_input_()
{
    // Strategy: Monitor grid input for power quality and availability
    // TODO: Implement grid input monitoring strategy
    
    ESP_LOGV(TAG, "⚡ Executing GRID_INPUT strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_inverter_ac_output_()
{
    // Strategy: Monitor inverter AC output and coordinate with other devices
    // TODO: Implement inverter AC output strategy
    
    ESP_LOGV(TAG, "🔌 Executing INVERTER_AC_OUTPUT strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_inverter_dc_battery_()
{
    // Strategy: Monitor battery state and manage charging/discharging
    // TODO: Implement battery management strategy
    
    ESP_LOGV(TAG, "🔋 Executing INVERTER_DC_BATTERY strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_inverter_dc_grid_power_()
{
    // Strategy: Monitor DC grid power and coordinate with battery
    // TODO: Implement DC grid power management strategy
    
    ESP_LOGV(TAG, "⚡ Executing INVERTER_DC_GRID_POWER strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_sinker_ac_heater_()
{
    // Strategy: Consume excess AC power through heater load
    // TODO: Implement AC heater power consumption strategy
    
    ESP_LOGV(TAG, "🔥 Executing SINKER_AC_HEATER strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_sinker_dc_heater_()
{
    // Strategy: Consume excess DC power through heater load
    // TODO: Implement DC heater power consumption strategy
    
    ESP_LOGV(TAG, "🔥 Executing SINKER_DC_HEATER strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_sinker_ac_vehicle_charger_()
{
    // Strategy: Consume excess AC power through vehicle charging
    // TODO: Implement AC vehicle charger power consumption strategy
    
    ESP_LOGV(TAG, "🚗 Executing SINKER_AC_VEHICLE_CHARGER strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_sinker_dc_vehicle_charger_()
{
    // Strategy: Consume excess DC power through vehicle charging
    // TODO: Implement DC vehicle charger power consumption strategy
    
    ESP_LOGV(TAG, "🚗 Executing SINKER_DC_VEHICLE_CHARGER strategy (not yet implemented)");
}

void PowerSyncComponent::strategy_solar_inverter_output_total_()
{
    // Strategy: Monitor INVERTER_AC_OUTPUT power to maintain it within configured range
    // Purpose: Adjust solar generation to keep inverter output power within [-min, +max] range
    // Use case: Balance solar generation with load demand to prevent grid feed or power shortage
    // State duration tracking is now handled by the generic update_state_duration_() method
    // 
    // Performance optimizations:
    // - Rate-limited logging for normal state (every 30s)
    // - Rate-limited event triggering for out-of-range state (every 5s)
    
    uint32_t now = millis();
    

    // Get INVERTER_AC_OUTPUT device state (grid inverter AC output)
    const DeviceState* inverter_output_state = this->get_device_state(ROLE_INVERTER_AC_OUTPUT);
    
    if (inverter_output_state == nullptr) {
        return;
    }
    
    // Check if data is fresh (within configured timeout)
    if (inverter_output_state->data_age_ms > this->power_decision_data_timeout_) {
        return;
    }
    
    // Get current power from inverter AC output
    float inverter_output_power = inverter_output_state->power;
    
    // Calculate power gap based on range
    // Power gap logic:
    // - If power > max: need to INCREASE solar generation (gap = power - max, positive value)
    // - If power < min: need to DECREASE solar generation (gap = power - min, negative value)
    // - If within range: gap = 0, no adjustment needed
    
    float power_gap = 0.0f;
    bool out_of_range = false;
    
    if (inverter_output_power > this->inverter_output_power_range_max_w_) {
        // Output power too high - need to increase solar generation
        // Example: output=200W (taking from grid), max=150W → gap=50W (need to increase solar by 50W)
        power_gap = inverter_output_power - this->inverter_output_power_range_max_w_;
        out_of_range = true;
    } else if (inverter_output_power < this->inverter_output_power_range_min_w_) {
        // Output power too low (or negative, feeding to grid) - need to decrease solar generation
        // Example: output=-200W (feeding to grid), min=-150W → gap=-50W (need to decrease solar by 50W)
        power_gap = inverter_output_power - this->inverter_output_power_range_min_w_;
        out_of_range = true;
    }
    
    if (out_of_range) {
        // Power is out of range - adjustment needed
        
        // Edge-triggered logging: Only log on state transition (in range → out of range)
        if (!this->last_inverter_output_out_of_range_) {
            ESP_LOGW(TAG, "⚡ Inverter AC output power OUT OF RANGE!");
            ESP_LOGW(TAG, "   Current output: %.2f W", inverter_output_power);
            ESP_LOGW(TAG, "   Configured range: [%.2f, %.2f] W", 
                     this->inverter_output_power_range_min_w_, 
                     this->inverter_output_power_range_max_w_);
            ESP_LOGW(TAG, "   Power gap: %.2f W", power_gap);
            
            if (power_gap > 0) {
                ESP_LOGW(TAG, "   → Need to INCREASE solar generation by %.2f W", power_gap);
            } else {
                ESP_LOGW(TAG, "   → Need to DECREASE solar generation by %.2f W", std::abs(power_gap));
            }
            
            // Trigger event immediately on state transition
            this->inverter_output_power_adjustment_callback_.call(power_gap);
            
            // Update state
            this->last_inverter_output_out_of_range_ = true;
            this->last_inverter_output_adjustment_log_time_ = now;
        } else {
            // Rate-limited event triggering and logging (every 5 seconds during persistent out-of-range)
            if (now - this->last_inverter_output_adjustment_log_time_ >= 5000) {
                ESP_LOGW(TAG, "⚠️ Still out of range - Output: %.2f W, Gap: %.2f W",
                         inverter_output_power, power_gap);
                
                // Trigger event again (rate-limited to every 5 seconds)
                this->inverter_output_power_adjustment_callback_.call(power_gap);
                
                this->last_inverter_output_adjustment_log_time_ = now;
            }
        }
    } else {
        // Power is within range - no adjustment needed
        
        // Edge-triggered logging: Log state transition (out of range → in range)
        if (this->last_inverter_output_out_of_range_) {
            ESP_LOGI(TAG, "✅ Inverter AC output power returned to normal range");
            ESP_LOGI(TAG, "   Current output: %.2f W", inverter_output_power);
            ESP_LOGI(TAG, "   Configured range: [%.2f, %.2f] W",
                     this->inverter_output_power_range_min_w_,
                     this->inverter_output_power_range_max_w_);
            
            // Update state
            this->last_inverter_output_out_of_range_ = false;
        }
        
        // Rate-limited verbose logging for normal state (every 30 seconds)
        static uint32_t last_in_range_log_time = 0;
        if (now - last_in_range_log_time >= 30000) {
            ESP_LOGI(TAG, "✅ Inverter AC output power within range: %.2f W (range: [%.2f, %.2f] W)",
                     inverter_output_power, 
                     this->inverter_output_power_range_min_w_,
                     this->inverter_output_power_range_max_w_);
            last_in_range_log_time = now;
        }
    }
}

// ============================================================================
// Generic State Duration Tracking (for all device roles)
// ============================================================================

void PowerSyncComponent::update_state_duration_()
{
    // Generic state duration tracking based on power sign
    // Works for all device roles that have power measurement
    // - Negative power (generation/reverse flow): accumulate negative duration
    // - Positive power (consumption/forward flow): accumulate positive duration
    // - Power sign change: reset duration to 0
    
    uint32_t now = millis();
    
    // Get own device state
    const DeviceState* own_state = this->get_device_state(this->device_role_);
    
    if (own_state == nullptr) {
        ESP_LOGV(TAG, "⚠️ Own device state not available - cannot track state duration");
        return;
    }
    
    // Check if data is fresh (within configured timeout)
    if (own_state->data_age_ms > this->power_decision_data_timeout_) {
        ESP_LOGV(TAG, "⚠️ Own device data is stale (age: %lu ms) - skipping state duration update",
                 own_state->data_age_ms);
        return;
    }
    
    float current_power = own_state->power;
    bool current_power_is_negative = (current_power < 0.0f);
    
    // Initialize on first run
    if (this->last_state_update_time_ == 0) {
        this->last_state_update_time_ = now;
        this->last_power_was_negative_ = current_power_is_negative;
        this->state_duration_seconds_ = 0;
        ESP_LOGI(TAG, "📊 State duration tracking initialized (power: %.2f W, sign: %s)",
                 current_power, current_power_is_negative ? "negative/generation" : "positive/consumption");
        return;
    }
    
    // Calculate time elapsed since last update (in seconds)
    uint32_t time_elapsed_ms = now - this->last_state_update_time_;
    int32_t time_elapsed_seconds = static_cast<int32_t>(time_elapsed_ms / 1000);
    
    // Skip if no significant time has passed (less than 1 second)
    if (time_elapsed_seconds < 1) {
        return;
    }
    
    // Check for power sign change (state transition)
    if (current_power_is_negative != this->last_power_was_negative_) {
        // State transition detected - log final duration before reset
        ESP_LOGI(TAG, "🔄 State transition detected:");
        ESP_LOGI(TAG, "   Previous state: %s (duration: %d seconds = %.2f hours)",
                 this->last_power_was_negative_ ? "generation (negative power)" : "consumption (positive power)",
                 std::abs(this->state_duration_seconds_),
                 std::abs(this->state_duration_seconds_) / 3600.0f);
        ESP_LOGI(TAG, "   New state: %s (power: %.2f W)",
                 current_power_is_negative ? "generation (negative power)" : "consumption (positive power)",
                 current_power);
        
        // Reset duration to 0 on state transition
        this->state_duration_seconds_ = 0;
        this->last_power_was_negative_ = current_power_is_negative;
        this->last_state_update_time_ = now;
        
        // Publish to sensor immediately
        if (this->state_duration_sensor_ != nullptr) {
            this->state_duration_sensor_->publish_state(0.0f);
            ESP_LOGI(TAG, "✅ State duration sensor updated: 0 hours (reset on transition)");
        }
        
        return;
    }
    
    // Same state continues - accumulate duration
    // Negative power -> accumulate negative duration (generation time)
    // Positive power -> accumulate positive duration (consumption time)
    if (current_power_is_negative) {
        this->state_duration_seconds_ -= time_elapsed_seconds;  // Accumulate negative
        ESP_LOGV(TAG, "📉 Generation state continues: power=%.2f W, duration=%d seconds (%.2f hours)",
                 current_power, this->state_duration_seconds_, this->state_duration_seconds_ / 3600.0f);
    } else {
        this->state_duration_seconds_ += time_elapsed_seconds;  // Accumulate positive
        ESP_LOGV(TAG, "📈 Consumption state continues: power=%.2f W, duration=%d seconds (%.2f hours)",
                 current_power, this->state_duration_seconds_, this->state_duration_seconds_ / 3600.0f);
    }
    
    // Update last update timestamp
    this->last_state_update_time_ = now;
    
    // Publish to sensor (convert seconds to hours for Home Assistant)
    if (this->state_duration_sensor_ != nullptr) {
        float duration_hours = this->state_duration_seconds_ / 3600.0f;
        this->state_duration_sensor_->publish_state(duration_hours);
        ESP_LOGI(TAG, "✅ State duration sensor updated: %.2f hours (%d seconds)",
                 duration_hours, this->state_duration_seconds_);
    }
}

void PowerSyncComponent::dump_device_states_table_()
{
    enum class ColumnAlign
    {
        Left,
        Right,
        Center
    };

    static constexpr size_t COLUMN_COUNT = 10;
    static const std::array<const char *, COLUMN_COUNT> COLUMN_HEADERS = {
        "Role ID",
        "Device Name",
        "Active",
        "Voltage",
        "Current",
        "Power",
        "RSSI",
        "Data Age",
        "Uptime",
        "Firmware"
    };
    static const std::array<const char *, COLUMN_COUNT> COLUMN_UNITS = {
        "",
        "",
        "",
        "(V)",
        "(A)",
        "(W)",
        "(dBm)",
        "(ms)",
        "(hrs)",
        ""
    };
    static const std::array<size_t, COLUMN_COUNT> COLUMN_WIDTHS = {
        7,   // Role ID
        25,  // Device Name (fits *_VEHICLE_CHARGER)
        6,   // Active
        7,   // Voltage
        7,   // Current
        8,   // Power
        5,   // RSSI
        9,   // Data Age
        9,   // Uptime
        18   // Firmware (slightly reduced to fit new column)
    };
    static const std::array<ColumnAlign, COLUMN_COUNT> DATA_ALIGNMENTS = {
        ColumnAlign::Left,
        ColumnAlign::Left,
        ColumnAlign::Center,
        ColumnAlign::Right,
        ColumnAlign::Right,
        ColumnAlign::Right,
        ColumnAlign::Right,
        ColumnAlign::Right,
        ColumnAlign::Right,
        ColumnAlign::Left
    };

    const auto make_uniform_alignment = [](ColumnAlign align) {
        std::array<ColumnAlign, COLUMN_COUNT> result{};
        result.fill(align);
        return result;
    };

    const std::array<ColumnAlign, COLUMN_COUNT> HEADER_ALIGNMENTS = make_uniform_alignment(ColumnAlign::Center);
    const std::array<ColumnAlign, COLUMN_COUNT> UNIT_ALIGNMENTS = make_uniform_alignment(ColumnAlign::Center);

    const auto format_text = [](const std::string &text, size_t width, ColumnAlign align) -> std::string {
        if (text.size() >= width) {
            return text.substr(0, width);
        }

        const size_t padding = width - text.size();
        switch (align) {
            case ColumnAlign::Left:
                return text + std::string(padding, ' ');
            case ColumnAlign::Right:
                return std::string(padding, ' ') + text;
            case ColumnAlign::Center: {
                const size_t left_padding = padding / 2;
                const size_t right_padding = padding - left_padding;
                return std::string(left_padding, ' ') + text + std::string(right_padding, ' ');
            }
        }
        return text;
    };

    size_t total_row_width = 1; // Leading separator
    for (size_t width : COLUMN_WIDTHS) {
        total_row_width += width + 3; // space + field + space + separator
    }

    const auto format_row = [&](const std::array<std::string, COLUMN_COUNT> &cells,
                                const std::array<ColumnAlign, COLUMN_COUNT> &alignments) -> std::string {
        std::string line;
        line.reserve(total_row_width);
        line.push_back('|');
        for (size_t idx = 0; idx < COLUMN_COUNT; ++idx) {
            line.push_back(' ');
            line += format_text(cells[idx], COLUMN_WIDTHS[idx], alignments[idx]);
            line.push_back(' ');
            line.push_back('|');
        }
        return line;
    };

    const auto make_border = [&](char fill_char) -> std::string {
        std::string line;
        line.reserve(total_row_width);
        line.push_back('+');
        for (size_t width : COLUMN_WIDTHS) {
            line.append(width + 2, fill_char);
            line.push_back('+');
        }
        return line;
    };

    const std::string outer_border = make_border('=');
    const std::string inner_border = make_border('-');
    const size_t line_width = outer_border.size();
    const size_t inner_width = (line_width > 2) ? (line_width - 2) : 0;

    const auto make_span_line = [&](const std::string &text, ColumnAlign align) -> std::string {
        std::string line(line_width, ' ');
        if (line_width == 0) {
            return line;
        }
        line.front() = '|';
        line.back() = '|';
        const size_t copy_len = std::min(text.size(), inner_width);

        size_t start = 1;
        if (align == ColumnAlign::Left) {
            start = 1;
        } else if (align == ColumnAlign::Right) {
            start = 1 + inner_width - copy_len;
        } else {
            start = 1 + (inner_width - copy_len) / 2;
        }

        line.replace(start, copy_len, text.substr(0, copy_len));
        return line;
    };

    std::array<std::string, COLUMN_COUNT> header_cells{};
    for (size_t idx = 0; idx < COLUMN_COUNT; ++idx) {
        header_cells[idx] = COLUMN_HEADERS[idx];
    }

    std::array<std::string, COLUMN_COUNT> unit_cells{};
    bool has_units = false;
    for (size_t idx = 0; idx < COLUMN_COUNT; ++idx) {
        if (COLUMN_UNITS[idx][0] != '\0') {
            unit_cells[idx] = COLUMN_UNITS[idx];
            has_units = true;
        }
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "%s", outer_border.c_str());
    const std::string title_line = make_span_line("POWERSYNC NETWORK DEVICE STATES TABLE", ColumnAlign::Center);
    ESP_LOGI(TAG, "%s", title_line.c_str());
    ESP_LOGI(TAG, "%s", outer_border.c_str());

    const std::string header_line = format_row(header_cells, HEADER_ALIGNMENTS);
    ESP_LOGI(TAG, "%s", header_line.c_str());

    if (has_units) {
        const std::string unit_line = format_row(unit_cells, UNIT_ALIGNMENTS);
        ESP_LOGI(TAG, "%s", unit_line.c_str());
    }

    ESP_LOGI(TAG, "%s", inner_border.c_str());
    
    uint32_t now = millis();
    int active_count = 0;
    
    for (size_t i = 0; i < MAX_DEVICE_ROLES; i++) {
        const DeviceState &state = this->device_states_[i];
        
        std::array<std::string, COLUMN_COUNT> row_cells{};
        row_cells[0] = "[" + std::to_string(static_cast<unsigned>(i)) + "]";
        row_cells[1] = tlv_device_role_to_string(i);
        
        const bool is_active = state.is_valid && (now - state.last_update_time <= DEVICE_STATE_TIMEOUT);
        row_cells[2] = is_active ? "YES" : "NO";
        
        if (is_active) {
            active_count++;
            
            char buffer[32];
            
            snprintf(buffer, sizeof(buffer), "%.1f", state.voltage);
            row_cells[3] = buffer;
            
            snprintf(buffer, sizeof(buffer), "%.3f", state.current);
            row_cells[4] = buffer;
            
            snprintf(buffer, sizeof(buffer), "%.2f", state.power);
            row_cells[5] = buffer;
            
            snprintf(buffer, sizeof(buffer), "%d", state.rssi);
            row_cells[6] = buffer;
            
            // Data age in milliseconds
            snprintf(buffer, sizeof(buffer), "%lu", state.data_age_ms);
            row_cells[7] = buffer;
            
            const float uptime_hours = state.uptime / 3600.0f;
            snprintf(buffer, sizeof(buffer), "%.2f", uptime_hours);
            row_cells[8] = buffer;
            
            if (!state.firmware_version.empty()) {
                row_cells[9] = state.firmware_version;
            } else {
                row_cells[9] = "N/A";
            }
        } else if (state.is_valid) {
            for (size_t idx = 3; idx <= 8; ++idx) {
                row_cells[idx] = "---";
            }
            
            const uint32_t time_since_update = (now - state.last_update_time) / 1000;  // seconds
            row_cells[9] = "timeout: " + std::to_string(static_cast<unsigned long>(time_since_update)) + "s ago";
        } else {
            for (size_t idx = 3; idx <= 8; ++idx) {
                row_cells[idx] = "---";
            }
            row_cells[9] = "not configured";
        }
        
        const std::string data_line = format_row(row_cells, DATA_ALIGNMENTS);
        ESP_LOGI(TAG, "%s", data_line.c_str());
    }
    
    ESP_LOGI(TAG, "%s", inner_border.c_str());
    
    char summary_buffer[160];
    snprintf(summary_buffer, sizeof(summary_buffer),
             "Summary: %d active device(s) out of %d total slots | Timeout: %d seconds",
             active_count, MAX_DEVICE_ROLES, DEVICE_STATE_TIMEOUT / 1000);
    
    const std::string summary_line = make_span_line(summary_buffer, ColumnAlign::Left);
    ESP_LOGI(TAG, "%s", summary_line.c_str());
    ESP_LOGI(TAG, "%s", outer_border.c_str());
    ESP_LOGI(TAG, "");
}

} // namespace powersync
} // namespace esphome
