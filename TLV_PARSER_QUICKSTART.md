# PowerSync TLV 流式解析器 - 快速入门

## 🎯 一分钟快速理解

**问题**：如何高效解析ESP-NOW接收到的TLV格式数据包？

**方案**：零拷贝流式解析 + 回调函数模式

```cpp
// 1. 定义回调函数处理每个TLV条目
bool my_callback(uint8_t type, uint8_t length, const uint8_t* value, void* ctx) {
    if (type == TLV_TYPE_AC_VOLTAGE) {
        float voltage;
        TLV_FLOAT32_FROM_BE(value, voltage);
        printf("电压: %.1f V\n", voltage);
    }
    return true; // 继续解析下一个
}

// 2. 调用解析器
int result = tlv_parse_stream(packet_data, packet_size, my_callback, NULL);

// 3. 检查结果
if (result >= 0) {
    printf("成功解析 %d 个TLV条目\n", result);
}
```

## 📦 核心API（3个函数）

### 1. `tlv_parse_stream()` - 主解析函数

```c
int tlv_parse_stream(const uint8_t* buffer, size_t buffer_size, 
                     tlv_parse_callback_t callback, void* user_data);
```

**返回值**：
- `>= 0`：成功解析的条目数
- `-1`：参数错误（空指针）
- `-2`：数据包格式错误
- `-3`：回调函数提前终止

### 2. `tlv_validate_packet()` - 快速验证

```c
bool tlv_validate_packet(const uint8_t* buffer, size_t buffer_size);
```

**用途**：解析前先验证数据包结构是否正确

### 3. `tlv_count_entries()` - 统计条目

```c
int tlv_count_entries(const uint8_t* buffer, size_t buffer_size);
```

**用途**：快速统计TLV包中有多少个条目

## 🚀 实际应用场景

### 场景1：显示所有接收到的数据

```cpp
bool display_all(uint8_t type, uint8_t length, const uint8_t* value, void* ctx) {
    const char* name = tlv_type_to_string(type);
    
    switch(type) {
        case TLV_TYPE_AC_VOLTAGE:
            float voltage;
            TLV_FLOAT32_FROM_BE(value, voltage);
            ESP_LOGI("TLV", "电压: %.1f V", voltage);
            break;
            
        case TLV_TYPE_DEVICE_ID:
            char id[17] = {0};
            memcpy(id, value, length);
            ESP_LOGI("TLV", "设备: %s", id);
            break;
            
        // ... 处理其他类型
    }
    
    return true; // 继续解析
}

// 使用
tlv_parse_stream(data, len, display_all, NULL);
```

### 场景2：查找特定设备

```cpp
struct FindDevice {
    const char* target_id;
    bool found;
};

bool find_device(uint8_t type, uint8_t length, const uint8_t* value, void* ctx) {
    FindDevice* context = (FindDevice*)ctx;
    
    if (type == TLV_TYPE_DEVICE_ID) {
        if (strncmp((char*)value, context->target_id, length) == 0) {
            context->found = true;
            return false; // 找到了，停止解析
        }
    }
    
    return true; // 继续查找
}

// 使用
FindDevice ctx = {"M5NanoC6", false};
tlv_parse_stream(data, len, find_device, &ctx);

if (ctx.found) {
    ESP_LOGI("TLV", "找到目标设备！");
}
```

### 场景3：统计平均值

```cpp
struct VoltageStats {
    float sum;
    int count;
};

bool collect_voltage(uint8_t type, uint8_t length, const uint8_t* value, void* ctx) {
    VoltageStats* stats = (VoltageStats*)ctx;
    
    if (type == TLV_TYPE_AC_VOLTAGE) {
        float voltage;
        TLV_FLOAT32_FROM_BE(value, voltage);
        stats->sum += voltage;
        stats->count++;
    }
    
    return true;
}

// 使用
VoltageStats stats = {0, 0};
tlv_parse_stream(data, len, collect_voltage, &stats);

if (stats.count > 0) {
    float avg = stats.sum / stats.count;
    ESP_LOGI("TLV", "平均电压: %.1f V", avg);
}
```

## 🔍 PowerSync组件中的集成

在 `powersync.cpp` 中，已经完全集成了TLV解析：

```cpp
case DATA_TLV_RECEIVED:
    ESP_LOGI(TAG, "收到ESP-NOW数据包");
    
    // 1. 验证数据包
    if (tlv_validate_packet(msg.body, msg.body_length)) {
        
        // 2. 统计条目数
        int count = tlv_count_entries(msg.body, msg.body_length);
        ESP_LOGI(TAG, "包含 %d 个TLV条目", count);
        
        // 3. 解析所有条目
        int parsed = tlv_parse_stream(
            msg.body, 
            msg.body_length, 
            component->tlv_parse_callback_,  // 内置回调
            component
        );
        
        ESP_LOGI(TAG, "成功解析 %d 个条目", parsed);
    }
    break;
```

内置的 `tlv_parse_callback_` 会自动：
- ✅ 识别18种TLV类型
- ✅ 自动格式化输出（电压V、电流A、功率W等）
- ✅ 显示emoji图标增强可读性
- ✅ 验证每个字段的长度
- ✅ 处理大小端转换

## 📊 支持的TLV类型一览表

| 类型码 | 名称 | 图标 | 数据格式 | 示例输出 |
|--------|------|------|----------|----------|
| 0x01 | UPTIME | ⏱️ | uint32 | `⏱️ Uptime: 3600 seconds (1.00 hours)` |
| 0x03 | DEVICE_ID | 🏷️ | string | `🏷️ Device ID: M5NanoC6-001` |
| 0x05 | MAC_ADDRESS | 🌐 | 6 bytes | `🌐 MAC: 12:34:56:78:9A:BC` |
| 0x07 | FREE_MEMORY | 💾 | uint32 | `💾 Free memory: 245760 bytes (240.0 KB)` |
| 0x08 | DEVICE_ROLE | 🎭 | uint8 | `🎭 Device role: GRID_INPUT (1)` |
| 0x10 | AC_VOLTAGE | ⚡ | float32 | `⚡ AC Voltage: 220.5 V` |
| 0x11 | AC_CURRENT | 🔌 | int32 | `🔌 AC Current: 2.350 A (2350 mA)` |
| 0x12 | AC_FREQUENCY | 📻 | float32 | `📻 AC Frequency: 50.02 Hz` |
| 0x13 | AC_POWER | 💡 | int32 | `💡 AC Power: 520.000 W (520000 mW)` |
| 0x50 | STATUS_FLAGS | 🚦 | uint16 | `🚦 Status Flags: 0x0021` |

完整列表见 `powersync_tlv_format.h`

## 🎨 辅助宏和函数

### 大小端转换

```c
// 从大端字节数组读取
uint32_t value = TLV_UINT32_FROM_BE(bytes);
float voltage;
TLV_FLOAT32_FROM_BE(bytes, voltage);

// 写入大端字节数组
TLV_UINT32_TO_BE(12345, bytes);
TLV_FLOAT32_TO_BE(220.5f, bytes);
```

### 定点数转换

```c
// 电流：毫安 ↔ 安培
int32_t ma = TLV_CURRENT_A_TO_MA(2.5f);   // 2.5A → 2500mA
float a = TLV_CURRENT_MA_TO_A(2500);      // 2500mA → 2.5A

// 功率：毫瓦 ↔ 瓦特
int32_t mw = TLV_POWER_W_TO_MW(520.0f);   // 520W → 520000mW
float w = TLV_POWER_MW_TO_W(520000);      // 520000mW → 520W
```

### 类型名称转换

```c
const char* name = tlv_type_to_string(0x01);        // "UPTIME"
const char* role = tlv_device_role_to_string(1);    // "GRID_INPUT"
```

## ⚡ 性能特点

| 特性 | 数值 | 说明 |
|------|------|------|
| 内存分配 | 0 bytes | 零动态分配 |
| 栈使用 | ~200 bytes | 函数调用栈 |
| 时间复杂度 | O(n) | 线性扫描 |
| 缓冲区拷贝 | 0次 | 零拷贝设计 |

## 🛡️ 安全保证

✅ 自动边界检查（防止缓冲区溢出）
✅ 畸形数据包检测
✅ 类型长度验证
✅ 空指针检查

## 📝 常见错误处理

```cpp
int result = tlv_parse_stream(data, len, callback, NULL);

switch (result) {
    case -1:
        ESP_LOGE(TAG, "错误：空指针或零长度");
        break;
    case -2:
        ESP_LOGE(TAG, "错误：数据包格式不正确");
        break;
    case -3:
        ESP_LOGW(TAG, "警告：回调函数提前终止");
        break;
    default:
        if (result >= 0) {
            ESP_LOGI(TAG, "成功解析 %d 个条目", result);
        }
        break;
}
```

## 🔧 测试建议

```cpp
// 测试1：正常数据包
uint8_t packet[] = {
    0x01, 0x04, 0x00, 0x00, 0x12, 0x34,  // UPTIME TLV
    0x10, 0x04, 0x43, 0x5C, 0x66, 0x66   // VOLTAGE TLV
};

int result = tlv_parse_stream(packet, sizeof(packet), callback, NULL);
assert(result == 2); // 应该解析出2个条目

// 测试2：验证功能
bool is_valid = tlv_validate_packet(packet, sizeof(packet));
assert(is_valid == true);

// 测试3：统计功能
int count = tlv_count_entries(packet, sizeof(packet));
assert(count == 2);
```

## 📚 完整文档

详细实现说明见：`TLV_STREAM_PARSER_IMPLEMENTATION.md`

---

**需要帮助？**
- 查看 `powersync_tlv_format.h` 中的详细注释
- 参考 `powersync.cpp` 中的 `tlv_parse_callback_` 实现
- 阅读完整文档 `TLV_STREAM_PARSER_IMPLEMENTATION.md`
