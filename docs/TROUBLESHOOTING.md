# Troubleshooting Guide

This guide helps diagnose and resolve common issues when using the VL53LX library with ESP32.

## Table of Contents
- [Communication Issues](#communication-issues)
- [Measurement Problems](#measurement-problems)
- [Performance Issues](#performance-issues)
- [Hardware Issues](#hardware-issues)
- [Build/Compilation Issues](#buildcompilation-issues)
- [Common Error Codes](#common-error-codes)
- [Debugging Tips](#debugging-tips)

## Communication Issues

### Problem: VL53LX_CommsInitialise() fails
**Symptoms:**
- Function returns VL53LX_ERROR_CONTROL_INTERFACE
- ESP32 logs show I2C errors
- Cannot establish communication with sensor

**Root Cause Analysis:**
1. **Verify I2C address format:**
   ```c
   // ST API expects 8-bit format (includes R/W bit)
   dev.i2c_slave_address = 0x52;  // Correct for ST API
   // NOT 0x29 (7-bit format used by some I2C libraries)
   ```

2. **Check physical connections:**
   ```
   VL53LX SDA → ESP32 GPIO8 (or custom pin)
   VL53LX SCL → ESP32 GPIO9 (or custom pin)
   VL53LX VDD → 3.3V (2.6V-3.5V range)
   VL53LX GND → GND
   ```

3. **Power supply validation:**
   - Measure actual VDD voltage under load (should be 3.3V ±10%)
   - Check current capability (minimum 50mA for safe operation)
   - Verify power supply noise is <50mV peak-to-peak
   - Ensure voltage is stable during sensor operation (±2% tolerance)

4. **I2C bus debugging:**
   ```c
   // Test basic I2C communication using ESP-IDF functions
   uint8_t test_data;
   esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, 0x29, 
       NULL, 0, &test_data, 1, pdMS_TO_TICKS(100));
   if (ret != ESP_OK) {
       ESP_LOGE(TAG, "I2C communication failed: %s", esp_err_to_name(ret));
   }
   
   // Check if device responds to its I2C address
   uint16_t model_id;
   if (VL53LX_RdWord(&dev, 0x010F, &model_id) == VL53LX_ERROR_NONE) {
       ESP_LOGI(TAG, "Device Model ID: 0x%04X (should be 0xEACC)", model_id);
   }
   ```

5. **Check I2C bus configuration:**
   ```c
   // Verify I2C bus setup matches library expectations
   ESP_LOGI(TAG, "I2C SDA pin: %d", VL53LX_I2C_SDA);
   ESP_LOGI(TAG, "I2C SCL pin: %d", VL53LX_I2C_SCL);
   ```

### Problem: Device not responding after power-on
**Symptoms:**
- VL53LX_WaitDeviceBooted() times out
- Communication works but device seems unresponsive

**Solutions:**
1. **Power sequence:**
   - Ensure VDD rises before enabling XSHUT
   - Wait at least 2ms after VDD stabilizes

2. **Use hardware reset:**
   ```c
   // Connect XSHUT to a GPIO and control it
   gpio_set_level(XSHUT_PIN, 0);  // Reset
   vTaskDelay(pdMS_TO_TICKS(10));
   gpio_set_level(XSHUT_PIN, 1);  // Release reset
   vTaskDelay(pdMS_TO_TICKS(10));
   ```

3. **Check power supply:**
   - VDD should be 2.6V to 3.5V (3.3V recommended)
   - Ensure sufficient current capability (20mA minimum)

## Measurement Problems

### Problem: No objects detected
**Symptoms:**
- `NumberOfObjectsFound` is always 0
- Sensor seems to be working but doesn't detect objects

**Solutions:**
1. **Check distance mode:**
   ```c
   // Try different distance modes
   VL53LX_SetDistanceMode(dev, VL53LX_DISTANCEMODE_SHORT);  // 0-1.3m
   VL53LX_SetDistanceMode(dev, VL53LX_DISTANCEMODE_MEDIUM); // 0-3m
   VL53LX_SetDistanceMode(dev, VL53LX_DISTANCEMODE_LONG);   // 0-4m
   ```

2. **Adjust timing budget:**
   ```c
   // Increase timing budget for better sensitivity
   VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 200000); // 200ms
   ```

3. **Check target properties:**
   - Target should be non-reflective to IR light
   - Avoid transparent, black, or highly reflective surfaces
   - Ensure target is within the sensor's field of view (27°)

4. **Ambient light conditions:**
   - Strong sunlight can interfere with measurements
   - Shield sensor from direct light sources

### Problem: Inaccurate distance readings
**Symptoms:**
- Measurements are consistently off by a fixed amount
- Distance readings fluctuate significantly

**Solutions:**
1. **Perform calibration:**
   ```c
   // Offset calibration at known distance
   VL53LX_PerformOffsetCalibration(dev, 100, &cal_data); // 100mm target
   
   // Crosstalk calibration (cover sensor)
   VL53LX_PerformXTalkCalibration(dev, &cal_data);
   ```

2. **Check for crosstalk:**
   - Ensure sensor window is clean
   - Check for reflections from nearby surfaces
   - Perform crosstalk calibration

3. **Temperature compensation:**
   - Large temperature changes can affect accuracy
   - Consider implementing temperature compensation if operating in extreme conditions

### Problem: Range status errors
**Symptoms:**
- `RangeStatus` indicates errors (not VL53LX_RANGESTATUS_RANGE_VALID)

**Common status codes and solutions:**
- **`VL53LX_RANGESTATUS_RANGE_VALID` (0)**: Range is valid and accurate
- **`VL53LX_RANGESTATUS_SIGMA_FAIL` (1)**: Increase timing budget, improve target reflectivity, or adjust sigma limit with `VL53LX_SetLimitCheckValue()`
- **`VL53LX_RANGESTATUS_SIGNAL_FAIL` (2)**: Move closer to target, increase timing budget, or check for range ignore threshold issues
- **`VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL` (4)**: Target too close/far for current distance mode - try different distance mode
- **`VL53LX_RANGESTATUS_HARDWARE_FAIL` (5)**: Check power supply, connections, and perform device reset
- **`VL53LX_RANGESTATUS_WRAP_TARGET_FAIL` (7)**: Wrapped target detection - may indicate multiple reflections
- **`VL53LX_RANGESTATUS_XTALK_SIGNAL_FAIL` (9)**: Excessive crosstalk - perform crosstalk calibration
- **`VL53LX_RANGESTATUS_SYNCRONISATION_INT` (10)**: First measurement after start - ignore this data
- **`VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE` (11)**: Valid but multiple pulses merged - acceptable for most applications

**Range Status Decision Tree:**
```c
switch (range_status) {
    case VL53LX_RANGESTATUS_RANGE_VALID:
    case VL53LX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
    case VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
        // Use measurement - these are acceptable
        break;
        
    case VL53LX_RANGESTATUS_SIGMA_FAIL:
        // Accuracy may be reduced - use with caution
        // Consider increasing timing budget
        break;
        
    case VL53LX_RANGESTATUS_SIGNAL_FAIL:
        // No valid target detected - do not use measurement
        break;
        
    case VL53LX_RANGESTATUS_SYNCRONISATION_INT:
        // First measurement - always ignore
        break;
        
    default:
        // Other errors - do not use measurement
        break;
}
```

## Performance Issues

### Problem: Slow measurement rate
**Symptoms:**
- Measurements take longer than expected
- System seems sluggish

**Solutions:**
1. **Optimize timing budget:**
   ```c
   // Reduce timing budget for faster measurements
   VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 50000); // 50ms
   ```

2. **Adjust inter-measurement period:**
   ```c
   // Reduce delay between measurements
   VL53LX_SetInterMeasurementPeriodMilliSeconds(dev, 50);
   ```

3. **Use interrupt mode:**
   - Implement interrupt-based measurement (see interrupt example)
   - Allows CPU to perform other tasks while waiting

### Problem: High CPU usage
**Symptoms:**
- System becomes unresponsive
- Other tasks starved of CPU time

**Solutions:**
1. **Use appropriate task delays:**
   ```c
   // Add delays in polling loops
   vTaskDelay(pdMS_TO_TICKS(10));
   ```

2. **Implement proper task priorities:**
   ```c
   // Lower priority for sensor tasks
   xTaskCreate(sensor_task, "sensor", 4096, NULL, 3, NULL);
   ```

3. **Use interrupt-driven approach:**
   - Reduces CPU overhead significantly
   - Allows for power-saving modes

## Hardware Issues

### Problem: Intermittent connection
**Symptoms:**
- Communication works sometimes but fails randomly
- Errors occur after system has been running for a while

**Solutions:**
1. **Check solder joints:**
   - Ensure all connections are solid
   - Look for cold solder joints or broken traces

2. **Wire length and quality:**
   - Keep I2C wires as short as possible
   - Use twisted pair for SDA/SCL if longer than 10cm
   - Add external pull-up resistors for longer connections

3. **Power supply stability:**
   - Check for voltage drops during operation
   - Add decoupling capacitors near the sensor

4. **Electromagnetic interference:**
   - Shield sensor from strong electromagnetic fields
   - Keep sensor away from switching power supplies

### Problem: GPIO1 interrupt not working
**Symptoms:**
- Interrupt-based examples don't work
- No interrupts received from sensor

**Solutions:**
1. **Check GPIO1 connection:**
   ```
   VL53LX GPIO1 → ESP32 GPIO pin (e.g., GPIO10)
   ```

2. **Verify interrupt configuration:**
   ```c
   .intr_type = GPIO_INTR_NEGEDGE  // VL53LX pulls low when ready
   ```

3. **Enable internal pull-up:**
   ```c
   .pull_up_en = GPIO_PULLUP_ENABLE
   ```

## Build/Compilation Issues

### Problem: Header files not found
**Symptoms:**
```
fatal error: vl53l3cx_api.h: No such file or directory
```

**Solutions:**
1. **Check library installation:**
   ```ini
   ; In platformio.ini
   lib_deps = mssaleh/ST_VL53LX@^0.1.0
   ```

2. **Verify include paths:**
   ```c
   #include "vl53l3cx_api.h"  // Correct
   #include "vl53lx/core/vl53lx_api.h"  // Alternative if needed
   ```

### Problem: Linker errors
**Symptoms:**
```
undefined reference to `VL53LX_CommsInitialise'
```

**Solutions:**
1. **Check framework setting:**
   ```ini
   framework = espidf  # Must be espidf, not arduino
   ```

2. **Verify platform:**
   ```ini
   platform = espressif32
   ```

### Problem: GPIO pin definition errors
**Symptoms:**
- Compilation errors related to GPIO definitions
- I2C initialization failures

**Solutions:**
1. **Use correct build flags:**
   ```ini
   build_flags = 
       -DVL53LX_I2C_SDA=8
       -DVL53LX_I2C_SCL=9
   ```

2. **Check GPIO availability:**
   - Ensure chosen GPIOs are available on your ESP32 variant

## Common Error Codes

| Error Code | Meaning | Common Causes | Solutions |
|------------|---------|---------------|-----------|
| `VL53LX_ERROR_NONE` | Success | - | - |
| `VL53LX_ERROR_CONTROL_INTERFACE` | Communication error | I2C issues, wiring | Check connections, verify I2C config |
| `VL53LX_ERROR_TIMEOUT` | Operation timeout | Device not responding | Check power, reset device |
| `VL53LX_ERROR_INVALID_PARAMS` | Invalid parameters | Wrong function parameters | Check API documentation |
| `VL53LX_ERROR_NOT_SUPPORTED` | Feature not supported | Using unsupported feature | Use alternative approach |

## Debugging Tips

### Enable detailed logging
```c
#include "esp_log.h"

// Set log level
esp_log_level_set("VL53LX", ESP_LOG_DEBUG);

// Add debug prints
ESP_LOGD("VL53LX", "Status: %d, Objects: %d", status, data.NumberOfObjectsFound);
```

### Monitor I2C traffic
```ini
; In platformio.ini
build_flags = 
    -DCORE_DEBUG_LEVEL=5
    -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG
```

### Use logic analyzer
- Connect logic analyzer to SDA/SCL lines
- Verify I2C transactions are correct
- Check for proper ACK/NACK responses

### Test with known good hardware
- Test with evaluation board first
- Compare with reference implementation
- Use ST's official tools for comparison

### Step-by-step debugging
1. Test basic I2C communication
2. Verify device ID reading
3. Test basic ranging
4. Add advanced features incrementally

### Memory debugging
```c
// Check stack usage
UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
ESP_LOGI("DEBUG", "Stack remaining: %d bytes", stack_remaining * sizeof(StackType_t));

// Check heap usage
ESP_LOGI("DEBUG", "Free heap: %d bytes", esp_get_free_heap_size());
```

## Getting Help

If problems persist:

1. **Check the examples:** Start with the basic example and work up to advanced features
2. **Review the API documentation:** Ensure you're using functions correctly
3. **Check ST's documentation:** Reference the official VL53LX documentation
4. **Community support:** Post issues on the GitHub repository with:
   - Complete error messages
   - Hardware setup description
   - Code snippets showing the problem
   - ESP-IDF version and board information
