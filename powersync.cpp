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

namespace esphome
{
namespace powersync
{

PowerSyncComponent *PowerSyncComponent::instance_ = nullptr;

void PowerSyncComponent::setup()
{
    ESP_LOGCONFIG(TAG, "Setting up PowerSync component...");

    // Set static instance for callbacks
    instance_ = this;

    // Initialize system info
    this->setup_system_info_();

    // Initialize ESP-NOW
    this->init_espnow_();

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
    ESP_LOGI(TAG, "Received ESP-NOW broadcast packet:");
    ESP_LOGI(TAG, "- Source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             src_addr[0], src_addr[1], src_addr[2], src_addr[3], src_addr[4], src_addr[5]);
    ESP_LOGI(TAG, "- Data length: %d bytes", len);
    ESP_LOGI(TAG, "- RSSI: %d dBm", rssi);

    // Log first few bytes of data for debugging
    if (len > 0) {
        std::string hex_data;
        for (int i = 0; i < std::min(len, 16); i++) {
            char hex_byte[4];
            snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
            hex_data += hex_byte;
        }
        ESP_LOGI(TAG, "- Data content (first 16 bytes): %s", hex_data.c_str());
    }

    ESP_LOGI(TAG, "📦 Trigger packet reception notification effect");
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
    payload.push_back(static_cast<uint8_t>(this->device_role_));
    ESP_LOGI(TAG, "- Device role: %d", static_cast<uint8_t>(this->device_role_));
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

// FreeRTOS task function implementation
void PowerSyncComponent::espnow_task_function_(void *pvParameters)
{
    PowerSyncComponent *component = static_cast<PowerSyncComponent *>(pvParameters);
    
    ESP_LOGI(TAG, "ESP-NOW task started");
    
    uint32_t last_broadcast_time = 0;
    uint32_t last_system_update_time = 0;
    
    while (true) {
        uint32_t now = millis();
        
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
        
        // Yield to other tasks - use shorter delay for responsive system updates
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
    }
}

} // namespace powersync
} // namespace esphome
