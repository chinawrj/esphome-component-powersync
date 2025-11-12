# INVERTER_AC_INPUT Strategy Complete Documentation

## Overview

The `strategy_inverter_ac_input_()` function is the intelligent relay control strategy for the inverter AC input in the PowerSync component. This strategy implements grid feed protection and emergency power restoration through a three-level priority detection mechanism.

## Function Switch

### Master Control Switch

```yaml
powersync:
  enable_inverter_ac_input_strategy: true  # Default: true
```

**Function**: Enable/disable the entire `strategy_inverter_ac_input_()` function
- `true`: Execute all three priority levels of detection and control
- `false`: Completely skip this strategy, no relay control

**Use Cases**: 
- Temporarily disable strategy during debugging
- Selective enablement in multi-device environments
- Isolate issues during troubleshooting

---

## Three-Level Priority Protection Mechanism

### Priority 1: Self Power Reverse Feed Protection (Highest Priority)

**Detection Target**: Monitor self (INVERTER_AC_INPUT) power value

**Trigger Condition**: Self power < 0 (reverse feed to grid)

**Relay Action**: 🔴 **TRIP** (Immediate disconnect)

**Features**:
- No need to wait for other data
- Immediate disconnect upon detecting reverse feed
- Edge-triggered, commands sent only on state transitions
- Reminder log every 5 seconds during persistent feed

**Configuration Parameters**: None (always enabled)

---

### Priority 2: Solar Inverter Coordination Protection

**Detection Target**: Monitor solar inverter (ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL) output power

**Prerequisites**: 
1. Self power ≥ 0 (or no data)
2. `check_solar_inverter_power = true`

**Trigger Condition**: Solar power < `solar_power_threshold`

**Relay Actions**: 
- Enter feed: 🔴 **TRIP** (Disconnect grid)
- Exit feed: 🟢 **CLOSE** (Restore connection)
- First detection: 🟢 **CLOSE** (State sync)

**Related Configuration Parameters**:

```yaml
powersync:
  check_solar_inverter_power: true          # Enable solar monitoring (default: false)
  solar_power_threshold: -10.0              # Feed trigger threshold (default: -10.0W)
  power_decision_data_timeout: 60000        # Data freshness timeout (default: 60s)
```

**Parameter Explanation**:
- `solar_power_threshold`: Negative value indicates feeding to grid, -10.0W allows up to 10W feed
- `power_decision_data_timeout`: Solar data older than this is considered expired

**Disabled Feature**: 
- ~~Force TRIP when data expires~~ (too aggressive, commented out)

---

### Priority 3: Emergency Power Restoration (NEW!)

**Detection Target**: Monitor availability of two power sources simultaneously
- Solar inverter output (ROLE_SOLOAR_INVERTER_OUTPUT_TOTAL)
- Inverter AC output (ROLE_INVERTER_AC_OUTPUT)

**Trigger Condition**: 
```
(Solar power < 1W OR Solar data invalid/expired) 
AND 
(Inverter output < 1W OR Inverter data invalid/expired)
```

**Relay Action**: 🟢 **CLOSE** (Emergency restore grid power)

**Physical Meaning**: 
When both inverter output and solar output cannot provide sufficient power (possible causes: nighttime, cloudy, device failure, restart), automatically switch to grid power to ensure uninterrupted load power.

**Related Configuration Parameters**:

```yaml
powersync:
  low_power_restore_threshold: 1.0          # Low power threshold (default: 1.0W)
  power_decision_data_timeout: 60000        # Data freshness timeout (default: 60s)
```

**Parameter Explanation**:
- `low_power_restore_threshold`: Power below this value is considered "no effective power supply"
- `power_decision_data_timeout`: Data older than this is considered "device offline"

**Features**:
- Dual power source redundancy detection
- Edge-triggered relay control
- Detailed status logging during power restoration
- Reminder log every 5 seconds during persistent low power

---

## Complete Configuration Examples

### Scenario 1: Conservative Configuration (Strict Protection + Fast Restore)

```yaml
powersync:
  device_role: INVERTER_AC_INPUT
  enable_inverter_ac_input_strategy: true   # Enable strategy
  
  # Priority 2: Solar protection
  check_solar_inverter_power: true
  solar_power_threshold: -5.0               # Strict: Only allow 5W feed
  
  # Priority 3: Emergency restore
  low_power_restore_threshold: 2.0          # Below 2W is considered invalid
  power_decision_data_timeout: 30000        # 30s fast timeout
```

**Use Case**: Zero tolerance for feed, fast power switching scenarios

---

### Scenario 2: Relaxed Configuration (Allow Small Feed + Stable Operation)

```yaml
powersync:
  device_role: INVERTER_AC_INPUT
  enable_inverter_ac_input_strategy: true
  
  # Priority 2: Solar protection
  check_solar_inverter_power: true
  solar_power_threshold: -40.0              # Relaxed: Allow 40W feed
  
  # Priority 3: Emergency restore
  low_power_restore_threshold: 1.0          # Default threshold
  power_decision_data_timeout: 120000       # 120s long timeout
```

**Use Case**: Avoid frequent switching at sunrise/sunset, device occasional restart doesn't affect

---

### Scenario 3: Self Protection Only (Disable Coordination)

```yaml
powersync:
  device_role: INVERTER_AC_INPUT
  enable_inverter_ac_input_strategy: true   # Enable strategy
  
  # Priority 2: Disable solar monitoring
  check_solar_inverter_power: false
  
  # Priority 3: Still enable emergency restore
  low_power_restore_threshold: 1.0
```

**Use Case**: Only rely on self power protection, don't care about solar status

---

### Scenario 4: Completely Disable Strategy

```yaml
powersync:
  device_role: INVERTER_AC_INPUT
  enable_inverter_ac_input_strategy: false  # Disable entire strategy
```

**Use Case**: Debugging, troubleshooting, manual relay control

---

## Relay Action Decision Table (Updated)

| Priority | Detection Source | Power Condition | Data State | Previous State | Relay Action | Configurable Parameters |
|-------|--------|---------|---------|---------|-----------|-----------|
| **Master Switch** | - | - | - | - | - | - |
| - | Strategy Switch | - | - | - | 🚫 **Skip All** | `enable_inverter_ac_input_strategy` |
| **P1** | **Self Power** | - | - | - | - | - |
| P1 | Self Power | `< 0` | Valid | `≠ FEEDING` | 🔴 **TRIP** | - |
| P1 | Self Power | `< 0` | Valid | `= FEEDING` | No Action (Log) | - |
| P1 | Self Power | `≥ 0` | Valid | Any | ⏭️ P2 | - |
| **P2** | **Solar Power** | - | - | - | - | - |
| P2 | Feature Switch | - | - | - | 🚫 **Skip P2** | `check_solar_inverter_power` |
| P2 | Solar Power | `< threshold` | `age ≤ timeout` | `≠ FEEDING` | 🔴 **TRIP** | `solar_power_threshold`<br>`power_decision_data_timeout` |
| P2 | Solar Power | `< threshold` | `age ≤ timeout` | `= FEEDING` | No Action (Log) | - |
| P2 | Solar Power | `≥ threshold` | `age ≤ timeout` | `= FEEDING` | 🟢 **CLOSE** | `solar_power_threshold` |
| P2 | Solar Power | `≥ threshold` | `age ≤ timeout` | `= INVALID` | 🟢 **CLOSE** | - |
| P2 | Solar Power | `≥ threshold` | `age ≤ timeout` | `= NORMAL` | No Action | - |
| P2 | Solar Power | Any | `age > timeout` | Any | ⚠️ Warning Log | - |
| **P3** | **Dual Power** | - | - | - | - | - |
| P3 | Solar+Inverter | Both `< threshold`<br>or both invalid | - | `≠ LOW_POWER` | 🟢 **CLOSE** | `low_power_restore_threshold`<br>`power_decision_data_timeout` |
| P3 | Solar+Inverter | Both low power | - | `= LOW_POWER` | No Action (Log) | - |
| P3 | Solar or Inverter | At least one normal | - | `= LOW_POWER` | ✅ State Restore | - |
| P3 | Solar or Inverter | At least one normal | - | `= INVALID` | ✅ Initialize | - |

---

## State Flow Diagram

```
                    ┌─────────────────────────┐
                    │  enable_inverter_ac_    │
                    │  input_strategy = true? │
                    └───────────┬─────────────┘
                                │
                      ┌─────────┴─────────┐
                      │ NO                │ YES
                      ▼                    ▼
                  🚫 Return        ┌──────────────────┐
                               │  Priority 1:      │
                               │  Self Power Check │
                               └────────┬──────────┘
                                        │
                              Self Power < 0 ?
                                        │
                        ┌───────────────┴───────────────┐
                        │ YES                           │ NO
                        ▼                                ▼
               ┌─────────────────┐          ┌──────────────────────┐
               │ State ≠ FEEDING?│          │  Priority 2:         │
               └────┬────────────┘          │  Solar Power Check   │
                    │                        │  (Optional)          │
         ┌──────────┴──────┐                └──────┬───────────────┘
         │ YES             │ NO                    │
         ▼                  ▼            check_solar... = true?
    🔴 TRIP          No Action(Log)                │
    Immediate Disconnect           ┌────────────┴────────────┐
                                   │ NO                      │ YES
                                   ▼                          ▼
                            ⏭️ Jump to P3      [Solar Detection Logic...]
                                                        │
                                                        ▼
                               ┌──────────────────────────────────┐
                               │  Priority 3:                     │
                               │  Emergency Power Restore         │
                               └────────┬─────────────────────────┘
                                        │
                    Solar AND Inverter both low/invalid?
                                        │
                        ┌───────────────┴───────────────┐
                        │ YES                           │ NO
                        ▼                                ▼
               ┌─────────────────┐          ┌──────────────────┐
               │ State ≠ LOW_PWR?│          │ At least one OK  │
               └────┬────────────┘          └───────┬──────────┘
                    │                               │
         ┌──────────┴──────┐                       ▼
         │ YES             │ NO            State Transition
         ▼                  ▼              (LOW_PWR → NORMAL)
    🟢 CLOSE        No Action(Log)
    Emergency Grid Restore
```

---

## Real-World Application Scenarios

### Scenario A: Daytime Normal Operation
1. ☀️ Solar generation sufficient (e.g., -200W feed)
2. 🔴 Priority 2 detects feed → TRIP
3. 🏠 Load powered by inverter
4. ✅ Priority 3 detects inverter output normal → No action

### Scenario B: Nighttime/Cloudy Switching
1. 🌙 Solar no output (0W)
2. 🔌 Inverter output also stops (battery depleted or fault)
3. ⚠️ Priority 3 detects dual power failure
4. 🟢 CLOSE relay → Switch to grid power
5. 💡 Load uninterrupted

### Scenario C: Inverter Maintenance/Restart
1. 🔧 Inverter restarting (output 0W)
2. 🌞 Solar also at night (0W)
3. ⚠️ Priority 3 detects dual low power
4. 🟢 Temporarily switch to grid
5. ✅ After inverter restart completes, Priority 2/3 automatically coordinate switch back

---

## Debugging Recommendations

### Step 1: Enable Detailed Logging
```yaml
logger:
  level: DEBUG
  logs:
    powersync: DEBUG
```

### Step 2: Phased Testing

**Phase 1: Test Master Switch**
```yaml
enable_inverter_ac_input_strategy: false  # Should have no action
```

**Phase 2: Test Priority 1 Only**
```yaml
enable_inverter_ac_input_strategy: true
check_solar_inverter_power: false  # Disable P2 and P3
```

**Phase 3: Test Priority 2**
```yaml
check_solar_inverter_power: true
solar_power_threshold: -5.0  # Strict testing
```

**Phase 4: Test Priority 3**
```yaml
low_power_restore_threshold: 10.0  # Raise threshold, easier to trigger
```

---

## FAQ

### Q1: Why does Priority 3 check two devices?
**A**: Double confirmation avoids false switching. If only checking one, it might switch to grid due to single device failure while another device is still supplying power normally.

### Q2: Will Priority 3 and Priority 2 conflict?
**A**: No. Priority 2 handles feed issues (TRIP), Priority 3 handles insufficient power (CLOSE). They complement each other.

### Q3: What if solar data is delayed?
**A**: Increase `power_decision_data_timeout` to allow more waiting time.

### Q4: Can I enable only Priority 3?
**A**: Yes, configure:
```yaml
check_solar_inverter_power: false  # Disable P2
enable_inverter_ac_input_strategy: true  # Keep P1 and P3
```

---

## Version History

- **v1.0.1** (2025-11-12): 
  - ✨ Added Priority 3 emergency power restoration feature
  - ✨ Added master control switch `enable_inverter_ac_input_strategy`
  - ✨ Added configuration parameter `low_power_restore_threshold`
  - 🐛 Disabled force TRIP when solar data expires (too aggressive)
  
- **v1.0.0**: Initial version (Priority 1 and Priority 2 only)
