# VL53LX API Documentation

This document provides an overview of the VL53LX library API for ESP-IDF projects.

## Table of Contents
- [Quick Start](#quick-start)
- [Core Functions](#core-functions)
- [Configuration Functions](#configuration-functions)
- [Measurement Functions](#measurement-functions)
- [Calibration Functions](#calibration-functions)
- [Error Handling](#error-handling)
- [Data Structures](#data-structures)
- [Examples](#examples)

## Quick Start

### 1. Include the Library
```c
#include "vl53lx_api.h"
```

### 2. Initialize Device Structure
```c
VL53LX_Dev_t dev;
VL53LX_DEV dev_handle = &dev;
dev.i2c_slave_address = 0x52;  // Default I2C address
```

### 3. Initialize Communication
```c
VL53LX_Error status = VL53LX_CommsInitialise(dev_handle, 0, 400);
if (status != VL53LX_ERROR_NONE) {
    // Handle error
}
```

### 4. Initialize Sensor
```c
status = VL53LX_WaitDeviceBooted(dev_handle);
status |= VL53LX_DataInit(dev_handle);
status |= VL53LX_SetDistanceMode(dev_handle, VL53LX_DISTANCEMODE_SHORT);
```

### 5. Start Measurements
```c
status = VL53LX_StartMeasurement(dev_handle);
```

### 6. Read Data
```c
VL53LX_MultiRangingData_t data;
status = VL53LX_GetMultiRangingData(dev_handle, &data);
```

## Core Functions

### VL53LX_CommsInitialise()
Initializes I2C communication with the sensor.

```c
VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t comms_type,
    uint16_t comms_speed_khz
);
```

**Parameters:**
- `pdev`: Pointer to device structure
- `comms_type`: Communication type (0 for I2C)
- `comms_speed_khz`: I2C speed in kHz (typically 400)

**Returns:** VL53LX_ERROR_NONE on success

### VL53LX_WaitDeviceBooted()
Waits for the device to complete its boot sequence.

```c
VL53LX_Error VL53LX_WaitDeviceBooted(VL53LX_Dev_t *pdev);
```

### VL53LX_DataInit()
Initializes the sensor's internal data structures and sets up default parameters.

```c
VL53LX_Error VL53LX_DataInit(VL53LX_Dev_t *pdev);
```

**Important Notes:**
- Must be called after `VL53LX_WaitDeviceBooted()`
- If called multiple times, calibration data must be restored using `VL53LX_SetOffsetCalibrationData()`
- May return `VL53LX_ERROR_CALIBRATION_WARNING` if incorrect calibration data is detected
- This function accesses the device via I2C

**Returns:** VL53LX_ERROR_NONE on success

### VL53LX_GetDeviceInfo()
Retrieves device information including model, type, and revision.

```c
VL53LX_Error VL53LX_GetDeviceInfo(
    VL53LX_Dev_t *pdev,
    VL53LX_DeviceInfo_t *pdevice_info
);
```

## Configuration Functions

### VL53LX_SetDistanceMode()
Sets the sensor's distance measurement mode.

```c
VL53LX_Error VL53LX_SetDistanceMode(
    VL53LX_Dev_t *pdev,
    VL53LX_DistanceModes distance_mode
);
```

**Distance Modes:**
- `VL53LX_DISTANCEMODE_SHORT`: Up to 1.3m, better ambient immunity, fastest measurements
- `VL53LX_DISTANCEMODE_MEDIUM`: Up to 3m, balanced performance and range  
- `VL53LX_DISTANCEMODE_LONG`: Up to 4m, best range performance, slower measurements

**Important Notes:**
- Distance mode affects maximum range, ambient light immunity, and measurement speed
- SHORT mode provides best performance in bright ambient light conditions
- LONG mode provides maximum range but is more susceptible to ambient light
- Choose based on your application's range requirements and lighting conditions

### VL53LX_SetMeasurementTimingBudgetMicroSeconds()
Sets the measurement timing budget in microseconds.

```c
VL53LX_Error VL53LX_SetMeasurementTimingBudgetMicroSeconds(
    VL53LX_Dev_t *pdev,
    uint32_t measurement_timing_budget_us
);
```

**Parameters:**
- `measurement_timing_budget_us`: Timing budget (20000 to 1000000 μs)

Longer timing budgets provide better accuracy and maximum range, while shorter budgets enable faster measurements.

**Timing Budget Guidelines:**
- 20ms (20000μs): Fastest measurements, reduced accuracy
- 50ms (50000μs): Good balance for most applications  
- 100ms (100000μs): High accuracy, good range performance
- 200ms+ (200000μs+): Maximum accuracy and range, slower update rate

**Performance vs. Timing Budget:**
- Shorter budgets: Higher measurement rate, lower accuracy, reduced maximum range
- Longer budgets: Lower measurement rate, higher accuracy, extended maximum range
- Optimal range depends on distance mode and environmental conditions

### VL53LX_SetInterMeasurementPeriodMilliSeconds()
Sets the time between measurements in continuous mode.

```c
VL53LX_Error VL53LX_SetInterMeasurementPeriodMilliSeconds(
    VL53LX_Dev_t *pdev,
    uint32_t inter_measurement_period_ms
);
```

### VL53LX_SetUserROI()
Configures a Region of Interest (ROI) for measurements.

```c
VL53LX_Error VL53LX_SetUserROI(
    VL53LX_Dev_t *pdev,
    VL53LX_UserRoi_t *puser_roi
);
```

ROI allows focusing on a specific area of the sensor's field of view for better precision.

## Measurement Functions

### VL53LX_StartMeasurement()
Starts continuous ranging measurements.

```c
VL53LX_Error VL53LX_StartMeasurement(VL53LX_Dev_t *pdev);
```

### VL53LX_StopMeasurement()
Stops ongoing measurements.

```c
VL53LX_Error VL53LX_StopMeasurement(VL53LX_Dev_t *pdev);
```

### VL53LX_GetMeasurementDataReady()
Checks if new measurement data is available.

```c
VL53LX_Error VL53LX_GetMeasurementDataReady(
    VL53LX_Dev_t *pdev,
    uint8_t *pdata_ready
);
```

### VL53LX_GetMultiRangingData()
Retrieves multi-object ranging data.

```c
VL53LX_Error VL53LX_GetMultiRangingData(
    VL53LX_Dev_t *pdev,
    VL53LX_MultiRangingData_t *pdata
);
```

### VL53LX_ClearInterruptAndStartMeasurement()
Clears the interrupt flag and starts the next measurement.

```c
VL53LX_Error VL53LX_ClearInterruptAndStartMeasurement(VL53LX_Dev_t *pdev);
```

This function should be called after reading measurement data in interrupt mode.

## Calibration Functions

### VL53LX_PerformRefSpadManagement()
Performs reference SPAD management calibration.

```c
VL53LX_Error VL53LX_PerformRefSpadManagement(VL53LX_Dev_t *pdev);
```

**Important Notes:**
- Should be performed once per device during manufacturing or first use
- Requires no external target - can be performed with sensor uncovered
- Results are automatically stored in device and used for subsequent measurements
- Critical for optimal sensor performance and accuracy

### VL53LX_PerformXTalkCalibration()
Performs crosstalk calibration to compensate for internal reflections.

```c
VL53LX_Error VL53LX_PerformXTalkCalibration(
    VL53LX_Dev_t *pdev,
    VL53LX_CalibrationData_t *pcalibration_data
);
```

**Procedure:**
- Cover the sensor completely (no external light or targets)
- Call this function to measure internal crosstalk
- Store calibration data in non-volatile memory
- Apply calibration data after each `VL53LX_DataInit()` call

### VL53LX_PerformOffsetCalibration()
Performs offset calibration at a known distance.

```c
VL53LX_Error VL53LX_PerformOffsetCalibration(
    VL53LX_Dev_t *pdev,
    int32_t cal_distance_mm,
    VL53LX_CalibrationData_t *pcalibration_data
);
```

**Procedure:**
- Place a flat, matte white target at a known distance (e.g., 100mm)
- Ensure target covers the entire sensor field of view
- Call this function with the exact target distance
- Store calibration data and apply after each `VL53LX_DataInit()`

**Calibration Best Practices:**
```c
// Typical calibration sequence during manufacturing
VL53LX_CalibrationData_t cal_data;

// 1. Reference SPAD management (uncovered sensor)
VL53LX_PerformRefSpadManagement(dev);

// 2. Crosstalk calibration (covered sensor)  
VL53LX_PerformXTalkCalibration(dev, &cal_data);

// 3. Offset calibration (100mm white target)
VL53LX_PerformOffsetCalibration(dev, 100, &cal_data);

// 4. Store cal_data in non-volatile memory

// 5. Apply calibration after each DataInit
VL53LX_DataInit(dev);
VL53LX_SetOffsetCalibrationData(dev, &cal_data);
VL53LX_SetXTalkCalibrationData(dev, &cal_data);
```

## Error Handling

All VL53LX functions return a `VL53LX_Error` code:

- `VL53LX_ERROR_NONE` (0): Success
- Negative values: Various error conditions

Common error codes:
- `VL53LX_ERROR_CONTROL_INTERFACE`: Communication error
- `VL53LX_ERROR_TIMEOUT`: Operation timeout
- `VL53LX_ERROR_INVALID_PARAMS`: Invalid parameters
- `VL53LX_ERROR_NOT_SUPPORTED`: Feature not supported

## Data Structures

### VL53LX_Dev_t
Device structure containing sensor configuration.

```c
typedef struct {
    uint8_t i2c_slave_address;  // I2C address (0x52 default)
    // Other internal fields...
} VL53LX_Dev_t;
```

### VL53LX_MultiRangingData_t
Structure containing measurement results.

```c
typedef struct {
    uint32_t TimeStamp;
    uint8_t StreamCount;
    uint8_t NumberOfObjectsFound;
    VL53LX_TargetRangeData_t RangeData[VL53LX_MAX_RANGE_RESULTS];
    uint8_t HasXtalkValueChanged;
    uint16_t EffectiveSpadRtnCount;
} VL53LX_MultiRangingData_t;
```

### VL53LX_TargetRangeData_t
Individual object measurement data.

```c
typedef struct {
    int16_t RangeMaxMilliMeter;         // Maximum detection distance
    int16_t RangeMinMilliMeter;         // Minimum detection distance
    FixPoint1616_t SignalRateRtnMegaCps;   // Signal rate (16.16 fixed point)
    FixPoint1616_t AmbientRateRtnMegaCps;  // Ambient rate (16.16 fixed point)
    FixPoint1616_t SigmaMilliMeter;        // Measurement uncertainty (16.16 fixed point)
    int16_t RangeMilliMeter;            // Distance in mm
    uint8_t RangeStatus;                // Measurement status
    uint8_t ExtendedRange;              // Extended range flag
} VL53LX_TargetRangeData_t;
```

### Range Status Values
Range status indicates the quality and validity of each measurement:

- **`VL53LX_RANGESTATUS_RANGE_VALID` (0)**: Valid range measurement
- **`VL53LX_RANGESTATUS_SIGMA_FAIL` (1)**: Sigma limit check failed - use `VL53LX_SetLimitCheckEnable()` to adjust
- **`VL53LX_RANGESTATUS_SIGNAL_FAIL` (2)**: Signal check failed - no target or range ignore threshold exceeded
- **`VL53LX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED` (3)**: Valid but minimum range clipped
- **`VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL` (4)**: Phase out of valid limits
- **`VL53LX_RANGESTATUS_HARDWARE_FAIL` (5)**: Hardware failure detected
- **`VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL` (6)**: Valid range but wrap-around check not performed
- **`VL53LX_RANGESTATUS_WRAP_TARGET_FAIL` (7)**: Wrapped target - no matching phase in other VCSEL period
- **`VL53LX_RANGESTATUS_PROCESSING_FAIL` (8)**: Processing failure
- **`VL53LX_RANGESTATUS_XTALK_SIGNAL_FAIL` (9)**: Crosstalk signal failure
- **`VL53LX_RANGESTATUS_SYNCRONISATION_INT` (10)**: First interrupt in back-to-back mode (ignore data)
- **`VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE` (11)**: Valid but result of merged pulses
- **`VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL` (12)**: Target present but insufficient signal
- **`VL53LX_RANGESTATUS_MIN_RANGE_FAIL` (13)**: Minimum range failure
- **`VL53LX_RANGESTATUS_RANGE_INVALID` (14)**: Invalid range
- **`VL53LX_RANGESTATUS_NONE` (255)**: No update - no meaningful data

## Examples

The library includes several examples:

1. **Basic Usage** (`examples/basic_usage/`): Simple polling-based distance measurement
2. **Advanced Features** (`examples/advanced_features/`): Multi-object detection and advanced configuration
3. **Interrupt Based** (`examples/interrupt_based/`): Efficient interrupt-driven operation

Each example includes:
- Complete source code (`main.c`)
- PlatformIO configuration (`platformio.ini`)
- Detailed comments explaining the functionality

## Platform-Specific Notes

### ESP32 Configuration
- Default I2C pins: SDA=GPIO8, SCL=GPIO9
- Override with build flags: `-DVL53LX_I2C_SDA=x -DVL53LX_I2C_SCL=y`
- Internal pull-ups enabled by default (45kΩ typical)
- Maximum I2C speed: 400 kHz (Fast Mode I2C)

### Memory Requirements
- Stack: Minimum 4KB per task using the sensor
- RAM: Approximately 2KB for sensor data structures
- Flash: Approximately 150KB for the complete library

### Performance Characteristics
- Measurement rates: 10-50 Hz depending on configuration
- Range accuracy: ±1% typical, ±3% maximum
- Multi-object detection: Up to 4 objects simultaneously
- Field of view: 27° full angle cone
- Operating temperature: -40°C to +85°C (industrial grade)

## Production Guidelines

### Critical Implementation Requirements

1. **Power Supply Design:**
   - Use clean 3.3V supply with <50mV ripple
   - Provide minimum 50mA current capability
   - Add 100nF ceramic decoupling capacitor near VDD pin
   - Consider 10µF bulk capacitor for noise immunity

2. **I2C Bus Design:**
   - Keep trace lengths <30cm for reliable operation
   - Use controlled impedance (50Ω) for longer traces
   - External pull-ups (4.7kΩ) recommended for >10cm traces
   - Avoid vias in I2C signal paths where possible

3. **Error Handling Strategy:**
   ```c
   // Always check return values
   VL53LX_Error status = VL53LX_GetMultiRangingData(dev, &data);
   if (status != VL53LX_ERROR_NONE) {
       // Handle error - don't use measurement data
       ESP_LOGE(TAG, "Measurement failed: %d", status);
       return;
   }
   
   // Check range status for each object
   if (data.RangeData[0].RangeStatus != VL53LX_RANGESTATUS_RANGE_VALID) {
       // Range status indicates potential issues
       ESP_LOGW(TAG, "Range status warning: %d", data.RangeData[0].RangeStatus);
   }
   ```

4. **Calibration for Production:**
   ```c
   // Perform reference SPAD management (one-time per device)
   VL53LX_PerformRefSpadManagement(dev);
   
   // Offset calibration at known distance (e.g., 100mm white target)
   VL53LX_PerformOffsetCalibration(dev, 100, &cal_data);
   
   // Store calibration data in non-volatile memory
   // Restore calibration data after each VL53LX_DataInit()
   VL53LX_SetOffsetCalibrationData(dev, &cal_data);
   ```

5. **Thermal Considerations:**
   - Allow for thermal drift in precision applications
   - Consider temperature compensation for ±1mm accuracy
   - Provide adequate heat dissipation (sensor draws ~20mA)

### Real-World Performance Optimization

1. **Target Surface Recommendations:**
   - Matte white surfaces: Best performance (>90% reflectivity)
   - Dark surfaces: Reduced range (use SHORT distance mode)
   - Reflective surfaces: May cause false readings
   - Transparent materials: Generally not detectable

2. **Environmental Considerations:**
   - Direct sunlight: Reduces maximum range significantly
   - High ambient light: Use shorter timing budgets
   - Dust/contamination: Clean sensor window regularly
   - Vibration: Secure mounting essential for stable readings

3. **Multi-Object Detection Best Practices:**
   - Objects must be separated by >5cm typically
   - Closer objects have higher signal strength
   - Check `NumberOfObjectsFound` before accessing data
   - Sort by signal strength for priority handling

### Debugging and Validation

1. **Signal Quality Assessment:**
   ```c
   if (data.RangeData[0].SignalRateRtnMegaCps < 1000) {
       ESP_LOGW(TAG, "Low signal: %u MCPS", data.RangeData[0].SignalRateRtnMegaCps);
   }
   if (data.RangeData[0].AmbientRateRtnMegaCps > 1000) {
       ESP_LOGW(TAG, "High ambient: %u MCPS", data.RangeData[0].AmbientRateRtnMegaCps);
   }
   ```

2. **Production Test Procedure:**
   - Verify I2C communication at power-on
   - Check device ID (0xEACC for VL53LX)
   - Perform ranging test at known distances
   - Validate measurement repeatability
   - Test under expected operating conditions
