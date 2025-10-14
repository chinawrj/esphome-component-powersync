#include "powersync.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include <cmath>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

namespace esphome
{
namespace powersync
{

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
        "PowerSyncESPNOW",                        // Task name
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
}

void PowerSyncComponent::update_system_info_()
{
    // Update uptime (in seconds)
    this->uptime_seconds_ = millis() / 1000;

    // Update free memory
    this->free_memory_ = esp_get_free_heap_size();
}

std::vector<uint8_t> PowerSyncComponent::build_tlv_packet_()
{
    std::vector<uint8_t> payload;

    ESP_LOGI(TAG, "Building composite TLV data packet containing all TLV types");

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

    ESP_LOGI(TAG, "Composite TLV data packet construction completed:");
    ESP_LOGI(TAG, "- Total packet length: %d bytes", payload.size());
    ESP_LOGI(TAG, "- Number of TLV types included: %d", 12);

    return payload;
}

void PowerSyncComponent::broadcast_tlv_data_()
{
    ESP_LOGI(TAG, "📡 ESP-NOW ready, sending TLV broadcast packet...");

    // Simulate AC measurements if simulation is enabled
    if (this->simulate_) {
        this->simulate_ac_measurements_();
    }

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
    if (this->rgb_power_enable_ != nullptr) {
        this->rgb_power_enable_->turn_on();
    }

    if (this->rgb_strip_ != nullptr) {
        auto call = this->rgb_strip_->turn_on();
        call.set_effect("Packet Received");
        call.set_brightness(0.9f);
        call.perform();
    }
}

void PowerSyncComponent::trigger_alert_red_effect_()
{
    if (this->rgb_power_enable_ != nullptr) {
        this->rgb_power_enable_->turn_on();
    }

    if (this->rgb_strip_ != nullptr) {
        auto call = this->rgb_strip_->turn_on();
        call.set_effect("Alert Red");
        call.set_brightness(1.0f);
        call.perform();
    }
}

// TLV helper methods implementation
void PowerSyncComponent::add_tlv_uptime_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding UPTIME TLV");
    this->add_tlv_uint32_(payload, TLV_TYPE_UPTIME, this->uptime_seconds_);
    ESP_LOGI(TAG, "- Uptime: %lu seconds", this->uptime_seconds_);
}

void PowerSyncComponent::add_tlv_device_id_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding DEVICE_ID TLV");
    this->add_tlv_string_(payload, TLV_TYPE_DEVICE_ID, this->device_id_);
    ESP_LOGI(TAG, "- Device ID: %s", this->device_id_.c_str());
}

void PowerSyncComponent::add_tlv_mac_address_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding MAC_ADDRESS TLV");
    this->add_tlv_bytes_(payload, TLV_TYPE_MAC_ADDRESS, this->mac_address_bytes_, 6);
    ESP_LOGI(TAG, "- MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
             this->mac_address_bytes_[0], this->mac_address_bytes_[1], this->mac_address_bytes_[2],
             this->mac_address_bytes_[3], this->mac_address_bytes_[4], this->mac_address_bytes_[5]);
}

void PowerSyncComponent::add_tlv_compile_time_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding COMPILE_TIME TLV");
    this->add_tlv_string_(payload, TLV_TYPE_COMPILE_TIME, this->compile_time_);
    ESP_LOGI(TAG, "- Compile time: %s", this->compile_time_.c_str());
}

void PowerSyncComponent::add_tlv_firmware_version_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding FIRMWARE_VER TLV");
    this->add_tlv_string_(payload, TLV_TYPE_FIRMWARE_VER, this->firmware_version_);
    ESP_LOGI(TAG, "- Firmware version: %s", this->firmware_version_.c_str());
}

void PowerSyncComponent::add_tlv_ac_voltage_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding AC_VOLTAGE TLV (from global variable)");
    this->add_tlv_float_(payload, TLV_TYPE_AC_VOLTAGE, this->tlv_ac_voltage_);
    ESP_LOGI(TAG, "- AC voltage: %.1f V", this->tlv_ac_voltage_);
}

void PowerSyncComponent::add_tlv_ac_current_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding AC_CURRENT TLV (fixed-point version)");
    this->add_tlv_int32_(payload, TLV_TYPE_AC_CURRENT, this->tlv_ac_current_ma_);
    ESP_LOGI(TAG, "- AC current: %d mA (%.3f A)", this->tlv_ac_current_ma_, this->tlv_ac_current_ma_ / 1000.0f);
}

void PowerSyncComponent::add_tlv_ac_frequency_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding AC_FREQUENCY TLV (from global variable)");
    this->add_tlv_float_(payload, TLV_TYPE_AC_FREQUENCY, this->tlv_ac_frequency_);
    ESP_LOGI(TAG, "- AC frequency: %.2f Hz", this->tlv_ac_frequency_);
}

void PowerSyncComponent::add_tlv_ac_power_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding AC_POWER TLV (fixed-point version)");
    this->add_tlv_int32_(payload, TLV_TYPE_AC_POWER, this->tlv_ac_power_mw_);
    ESP_LOGI(TAG, "- AC power: %d mW (%.3f W)", this->tlv_ac_power_mw_, this->tlv_ac_power_mw_ / 1000.0f);
}

void PowerSyncComponent::add_tlv_device_role_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding DEVICE_ROLE TLV");
    payload.push_back(TLV_TYPE_DEVICE_ROLE);
    payload.push_back(1); // Length: 1 byte
    uint8_t role_value = static_cast<uint8_t>(this->device_role_);
    payload.push_back(role_value);
    
    // Use shared TLV helper function from powersync_tlv_format.h (C-compatible)
    ESP_LOGI(TAG, "- Device role: %s (%d)", 
             tlv_device_role_to_string(role_value), 
             role_value);
}

void PowerSyncComponent::add_tlv_free_memory_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding FREE_MEMORY TLV");
    this->add_tlv_uint32_(payload, TLV_TYPE_FREE_MEMORY, this->free_memory_);
    ESP_LOGI(TAG, "- Free memory: %lu Bytes (%.1f KB)", this->free_memory_, this->free_memory_ / 1024.0f);
}

void PowerSyncComponent::add_tlv_status_flags_(std::vector<uint8_t> &payload)
{
    ESP_LOGI(TAG, "Adding STATUS_FLAGS TLV");
    uint16_t status_flags = STATUS_POWER_ON | STATUS_ESP_NOW_ACTIVE;
    payload.push_back(TLV_TYPE_STATUS_FLAGS);
    payload.push_back(2); // Length
    payload.push_back((status_flags >> 8) & 0xFF);
    payload.push_back(status_flags & 0xFF);
    ESP_LOGI(TAG, "- Status flags: 0x%04X", status_flags);
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
    this->tlv_ac_voltage_ = voltage;
}

void PowerSyncComponent::set_ac_current(float current)
{
    this->tlv_ac_current_ma_ = static_cast<int32_t>(current * 1000.0f);
}

void PowerSyncComponent::set_ac_frequency(float frequency)
{
    this->tlv_ac_frequency_ = frequency;
}

void PowerSyncComponent::set_ac_power(float power)
{
    this->tlv_ac_power_mw_ = static_cast<int32_t>(power * 1000.0f);
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

void PowerSyncComponent::simulate_ac_measurements_()
{
    ESP_LOGI(TAG, "🎲 Simulating AC measurements with reasonable random values...");
    
    // Get current time for seed variation (use smaller numbers to avoid overflow)
    uint32_t time_sec = millis() / 1000;  // Convert to seconds for slower variation
    
    // Generate reasonable AC voltage (220V ± 10V)
    float base_voltage = 220.0f;
    int32_t voltage_offset = (int32_t)(time_sec % 200) - 100;  // -100 to +100
    float voltage_variation = (float)voltage_offset * 0.1f;    // ±10V variation
    this->tlv_ac_voltage_ = base_voltage + voltage_variation;
    
    // Generate reasonable AC current (-5A to +5A, allowing negative for reverse power flow)
    float base_current = 0.0f;  // Center around 0A
    int32_t current_offset = (int32_t)((time_sec + 50) % 100) - 50;  // -50 to +50
    float current_variation = (float)current_offset * 0.1f;           // ±5A variation
    float simulated_current = base_current + current_variation;
    this->tlv_ac_current_ma_ = static_cast<int32_t>(simulated_current * 1000.0f);
    
    // Generate reasonable AC frequency (49.8Hz to 50.2Hz)
    float base_frequency = 50.0f;
    int32_t frequency_offset = (int32_t)((time_sec + 25) % 40) - 20;  // -20 to +20
    float frequency_variation = (float)frequency_offset * 0.01f;      // ±0.2Hz variation
    this->tlv_ac_frequency_ = base_frequency + frequency_variation;
    
    // Calculate power from voltage and current (with some power factor simulation)
    // Power factor should be positive, but power can be negative if current is negative
    int32_t pf_offset = (int32_t)((time_sec + 75) % 30) - 15;        // -15 to +15
    float power_factor = 0.85f + (float)pf_offset * 0.01f;           // 0.7 to 1.0 power factor
    if (power_factor < 0.7f) power_factor = 0.7f;
    if (power_factor > 1.0f) power_factor = 1.0f;
    
    // Power = Voltage × Current × Power_Factor (can be negative for reverse power flow)
    float simulated_power = this->tlv_ac_voltage_ * simulated_current * power_factor;
    this->tlv_ac_power_mw_ = static_cast<int32_t>(simulated_power * 1000.0f);
    
    ESP_LOGI(TAG, "🎲 Simulated values:");
    ESP_LOGI(TAG, "   - AC Voltage: %.1f V", this->tlv_ac_voltage_);
    ESP_LOGI(TAG, "   - AC Current: %.3f A (%d mA)", simulated_current, this->tlv_ac_current_ma_);
    ESP_LOGI(TAG, "   - AC Frequency: %.2f Hz", this->tlv_ac_frequency_);
    ESP_LOGI(TAG, "   - AC Power: %.3f W (%d mW)", simulated_power, this->tlv_ac_power_mw_);
    ESP_LOGI(TAG, "   - Power Factor: %.2f", power_factor);
}

// TLV parsing callback implementation
bool PowerSyncComponent::tlv_parse_callback_(uint8_t type, uint8_t length, const uint8_t* value, void* user_data)
{
    // user_data is the TLVParseContext pointer
    TLVParseContext* ctx = static_cast<TLVParseContext*>(user_data);
    
    // Use TLV helper function to get type name
    const char* type_name = tlv_type_to_string(type);
    
    ESP_LOGI(TAG, "📦 TLV Entry: Type=0x%02X (%s), Length=%d", type, type_name, length);
    
    // Parse different TLV types and store in context
    switch (type) {
        case TLV_TYPE_UPTIME: {
            if (length == TLV_SIZE_UPTIME) {
                ctx->device_state.uptime = TLV_UINT32_FROM_BE(value);
                ctx->has_uptime = true;
                ESP_LOGI(TAG, "   ⏱️  Uptime: %lu seconds (%.2f hours)", 
                         ctx->device_state.uptime, ctx->device_state.uptime / 3600.0f);
            } else {
                ESP_LOGW(TAG, "   ⚠️  Invalid uptime length: %d (expected %d)", length, TLV_SIZE_UPTIME);
            }
            break;
        }
        
        case TLV_TYPE_TIMESTAMP: {
            if (length == TLV_SIZE_TIMESTAMP) {
                uint32_t timestamp = TLV_UINT32_FROM_BE(value);
                ESP_LOGI(TAG, "   🕐 Timestamp: %lu", timestamp);
            }
            break;
        }
        
        case TLV_TYPE_DEVICE_ID: {
            if (length > 0 && length <= TLV_MAX_DEVICE_ID_LEN) {
                char device_id[TLV_MAX_DEVICE_ID_LEN + 1] = {0};
                memcpy(device_id, value, length);
                ESP_LOGI(TAG, "   🏷️  Device ID: %s", device_id);
            }
            break;
        }
        
        case TLV_TYPE_FIRMWARE_VER: {
            if (length > 0 && length <= TLV_MAX_FIRMWARE_VER_LEN) {
                char firmware[TLV_MAX_FIRMWARE_VER_LEN + 1] = {0};
                memcpy(firmware, value, length);
                ctx->device_state.firmware_version = std::string(firmware);
                ctx->has_firmware = true;
                ESP_LOGI(TAG, "   📌 Firmware: %s", firmware);
            }
            break;
        }
        
        case TLV_TYPE_MAC_ADDRESS: {
            if (length == TLV_SIZE_MAC_ADDRESS) {
                ESP_LOGI(TAG, "   🌐 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                        value[0], value[1], value[2], value[3], value[4], value[5]);
            }
            break;
        }
        
        case TLV_TYPE_COMPILE_TIME: {
            if (length > 0 && length <= TLV_MAX_COMPILE_TIME_LEN) {
                char compile_time[TLV_MAX_COMPILE_TIME_LEN + 1] = {0};
                memcpy(compile_time, value, length);
                ESP_LOGI(TAG, "   🔨 Compile time: %s", compile_time);
            }
            break;
        }
        
        case TLV_TYPE_FREE_MEMORY: {
            if (length == TLV_SIZE_FREE_MEMORY) {
                uint32_t free_mem = TLV_UINT32_FROM_BE(value);
                ESP_LOGI(TAG, "   💾 Free memory: %lu bytes (%.1f KB)", free_mem, free_mem / 1024.0f);
            }
            break;
        }
        
        case TLV_TYPE_DEVICE_ROLE: {
            if (length == TLV_SIZE_DEVICE_ROLE) {
                uint8_t role = value[0];
                ctx->device_state.role = static_cast<DeviceRole>(role);
                ctx->has_role = true;
                const char* role_name = tlv_device_role_to_string(role);
                ESP_LOGI(TAG, "   🎭 Device role: %s (%d)", role_name, role);
            }
            break;
        }
        
        case TLV_TYPE_AC_VOLTAGE: {
            if (length == TLV_SIZE_AC_VOLTAGE) {
                TLV_FLOAT32_FROM_BE(value, ctx->device_state.voltage);
                ctx->has_voltage = true;
                ESP_LOGI(TAG, "   ⚡ AC Voltage: %.1f V", ctx->device_state.voltage);
            }
            break;
        }
        
        case TLV_TYPE_AC_CURRENT: {
            if (length == TLV_SIZE_AC_CURRENT) {
                int32_t current_ma = TLV_INT32_FROM_BE(value);
                ctx->device_state.current = TLV_CURRENT_MA_TO_A(current_ma);
                ctx->has_current = true;
                ESP_LOGI(TAG, "   🔌 AC Current: %.3f A (%d mA)", ctx->device_state.current, current_ma);
            }
            break;
        }
        
        case TLV_TYPE_AC_FREQUENCY: {
            if (length == TLV_SIZE_AC_FREQUENCY) {
                float frequency;
                TLV_FLOAT32_FROM_BE(value, frequency);
                ESP_LOGI(TAG, "   📻 AC Frequency: %.2f Hz", frequency);
            }
            break;
        }
        
        case TLV_TYPE_AC_POWER: {
            if (length == TLV_SIZE_AC_POWER) {
                int32_t power_mw = TLV_INT32_FROM_BE(value);
                ctx->device_state.power = TLV_POWER_MW_TO_W(power_mw);
                ctx->has_power = true;
                ESP_LOGI(TAG, "   💡 AC Power: %.3f W (%d mW)", ctx->device_state.power, power_mw);
            }
            break;
        }
        
        case TLV_TYPE_AC_POWER_FACTOR: {
            if (length == TLV_SIZE_AC_POWER_FACTOR) {
                float power_factor;
                TLV_FLOAT32_FROM_BE(value, power_factor);
                ESP_LOGI(TAG, "   📊 Power Factor: %.3f", power_factor);
            }
            break;
        }
        
        case TLV_TYPE_ENERGY_TOTAL: {
            if (length == TLV_SIZE_ENERGY_TOTAL) {
                float energy;
                TLV_FLOAT32_FROM_BE(value, energy);
                ESP_LOGI(TAG, "   🔋 Total Energy: %.2f kWh", energy);
            }
            break;
        }
        
        case TLV_TYPE_ENERGY_TODAY: {
            if (length == TLV_SIZE_ENERGY_TODAY) {
                float energy;
                TLV_FLOAT32_FROM_BE(value, energy);
                ESP_LOGI(TAG, "   📅 Today's Energy: %.2f kWh", energy);
            }
            break;
        }
        
        case TLV_TYPE_STATUS_FLAGS: {
            if (length == TLV_SIZE_STATUS_FLAGS) {
                uint16_t flags = TLV_UINT16_FROM_BE(value);
                ESP_LOGI(TAG, "   🚦 Status Flags: 0x%04X", flags);
                if (flags & STATUS_FLAG_POWER_ON) ESP_LOGI(TAG, "      - Power ON");
                if (flags & STATUS_FLAG_CALIBRATING) ESP_LOGI(TAG, "      - Calibrating");
                if (flags & STATUS_FLAG_ERROR) ESP_LOGI(TAG, "      - Error");
                if (flags & STATUS_FLAG_LOW_BATTERY) ESP_LOGI(TAG, "      - Low Battery");
                if (flags & STATUS_FLAG_WIFI_CONNECTED) ESP_LOGI(TAG, "      - WiFi Connected");
                if (flags & STATUS_FLAG_ESP_NOW_ACTIVE) ESP_LOGI(TAG, "      - ESP-NOW Active");
            }
            break;
        }
        
        case TLV_TYPE_ERROR_CODE: {
            if (length == TLV_SIZE_ERROR_CODE) {
                uint16_t error_code = TLV_UINT16_FROM_BE(value);
                ESP_LOGI(TAG, "   ❌ Error Code: 0x%04X", error_code);
            }
            break;
        }
        
        case TLV_TYPE_TEMPERATURE: {
            if (length == TLV_SIZE_TEMPERATURE) {
                float temperature;
                TLV_FLOAT32_FROM_BE(value, temperature);
                ESP_LOGI(TAG, "   🌡️  Temperature: %.1f °C", temperature);
            }
            break;
        }
        
        case TLV_TYPE_HUMIDITY: {
            if (length == TLV_SIZE_HUMIDITY) {
                float humidity;
                TLV_FLOAT32_FROM_BE(value, humidity);
                ESP_LOGI(TAG, "   💧 Humidity: %.1f %%", humidity);
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
                    
                case DATA_TLV_RECEIVED:
                    ESP_LOGI(TAG, "Processing DATA_TLV_RECEIVED message from queue");
                    ESP_LOGI(TAG, "- Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                             msg.src_addr[0], msg.src_addr[1], msg.src_addr[2], 
                             msg.src_addr[3], msg.src_addr[4], msg.src_addr[5]);
                    ESP_LOGI(TAG, "- Data length: %zu bytes", msg.body_length);
                    ESP_LOGI(TAG, "- RSSI: %d dBm", msg.rssi);
                    
                    // Validate TLV packet structure before parsing
                    if (tlv_validate_packet(msg.body, msg.body_length)) {
                        ESP_LOGI(TAG, "✅ TLV packet structure is valid");
                        
                        // Count TLV entries
                        int entry_count = tlv_count_entries(msg.body, msg.body_length);
                        ESP_LOGI(TAG, "📊 TLV packet contains %d entries", entry_count);
                        
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
                            ESP_LOGI(TAG, "✅ Successfully parsed %d TLV entries", parsed);
                            
                            // Update device state if we have valid role and at least one measurement
                            if (ctx.has_role && (ctx.has_voltage || ctx.has_current || ctx.has_power)) {
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
                                
                                ESP_LOGI(TAG, "💾 Updated device state for role %s (V:%d I:%d P:%d U:%d F:%d)", 
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
        
        // Update system info at specified interval
        if (now - last_system_update_time >= component->system_update_interval_) {
            component->update_system_info_();
            last_system_update_time = now;
        }
        
        // Broadcast TLV data at specified interval
        if (component->espnow_ready_ && now - last_broadcast_time >= component->broadcast_interval_) {
            component->broadcast_tlv_data_();
            last_broadcast_time = now;
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

} // namespace powersync
} // namespace esphome
