# ST VL53LX library for ESP‑IDF / PlatformIO

This library packages STMicroelectronics’ official VL53LX time‑of‑flight (ToF) ranging API together with a small platform port layer for the ESP‑IDF framework. It allows developers to integrate the VL53LX sensor into ESP32 projects using PlatformIO. The underlying driver code is taken directly from the ST API and retains its original dual licence (GPL‑2.0 or BSD‑3‑Clause).

## Features

- **Full ST API:** All header and source files from the official ST VL53LX "BareDriver" are included. Only the low‑level platform stubs have been replaced with an implementation for ESP‑IDF.
- **ESP‑IDF support:** Communication is implemented using the new ESP‑IDF I2C master driver. The code follows the examples provided in the ESP‑IDF programming guide for master write and read transactions.
- **Non‑blocking delays:** Microsecond delays use `esp_rom_delay_us()`, while millisecond delays use FreeRTOS `vTaskDelay()`. The tick counter functions rely on `esp_timer_get_time()` for millisecond precision.
- **Enhanced GPIO control:** Full implementation of GPIO functions for reset (XSHUT), power control, and interrupt handling using ESP32 GPIO API.
- **ESP-IDF logging integration:** VL53LX internal logging is integrated with ESP-IDF's unified logging system for consistent log formatting and level control.
- **Multi-object detection:** Supports advanced features like multi-object detection, various distance modes, and customizable ROI (Region of Interest).

## Installation

1. Add the repository as a dependency in your PlatformIO project. In your project’s `platformio.ini` file:

   ```ini
   [env:esp32-c6-devkitm-1]
   platform = espressif32
   framework = espidf
   board = esp32-c6-devkitm-1
   lib_deps =
     mssaleh/ST_VL53LX@^0.1.0
   ```

   The board identifier `esp32-c6-devkitm-1` corresponds to Espressif’s ESP32-C6-Mini development kit. Ensure that your PlatformIO platform and framework versions are aligned with ESP‑IDF v5.4.2 or later. Default GPIO assignments are provided (SDA on GPIO 8, SCL on GPIO 9) but can be overridden at build time. Internal pull‑ups are enabled by default as recommended in the ESP‑IDF documentation.

2. Optionally override the default I2C pins by defining preprocessor macros in your `platformio.ini` under `build_flags`:

   ```ini
   build_flags =
     -DVL53LX_I2C_SDA=8     ; I2C SDA pin (default: GPIO8)
     -DVL53LX_I2C_SCL=9     ; I2C SCL pin (default: GPIO9)  
     -DVL53LX_XSHUT_PIN=11  ; Reset pin (default: GPIO11)
     -DVL53LX_INT_PIN=10    ; Interrupt pin (default: GPIO10)
     -DVL53LX_PWR_PIN=12    ; Power control pin (default: GPIO12)
     -DVL53LX_LOG_ENABLE    ; Enable VL53LX internal logging
   ```

   Replace `5` and `6` with the GPIO numbers connected to the sensor’s SDA and SCL pins. By default the library uses GPIO 8 for SDA and GPIO 9 for SCL.

3. Build your project as usual. PlatformIO will automatically fetch this library and compile the ST driver sources.

## Hardware Setup

### Basic Connection (I2C only)
- Connect VL53LX **VDD** to **3.3V**
- Connect VL53LX **GND** to **GND**  
- Connect VL53LX **SDA** to **GPIO 8** (configurable via VL53LX_I2C_SDA)
- Connect VL53LX **SCL** to **GPIO 9** (configurable via VL53LX_I2C_SCL)

### Advanced Connection (with GPIO control)
For full functionality including reset control, power management, and interrupt handling:
- Connect VL53LX **XSHUT** to **GPIO 11** (reset control, configurable via VL53LX_XSHUT_PIN)
- Connect VL53LX **GPIO1** to **GPIO 10** (interrupt pin, configurable via VL53LX_INT_PIN)
- Optional: Connect power control circuit to **GPIO 12** (configurable via VL53LX_PWR_PIN)

### I2C Pull-up Resistors
The library enables ESP32 internal pull-up resistors by default. For longer I2C bus runs or multiple devices, external 4.7kΩ pull-up resistors to 3.3V may be required on SDA and SCL lines.

## Critical Calibration Requirements

**⚠️ IMPORTANT**: For optimal performance, the VL53LX sensor requires factory calibration to be performed **once** during production. The calibration must be done in this exact order:

1. **RefSPAD Calibration** (no target required)
2. **Crosstalk Calibration** (600mm target in dark environment) 
3. **Offset Calibration** (known distance target)

**CRITICAL**: Crosstalk compensation is **DISABLED by default** and must be explicitly enabled after loading calibration data:

```c
// After loading calibration data, MUST enable crosstalk compensation
VL53LX_SetXTalkCompensationEnable(&device, 1);
```

See the [Factory Calibration](#factory-calibration) section below for complete procedures.

## Usage example

The library exposes all functions provided by STMicroelectronics through the `vl53l3cx_api.h` header. A minimal example that initialises the sensor, starts a ranging measurement and reads the distance is shown below. Save this file under `src/main.c` in your PlatformIO project.

**⚠️ NOTE**: This example assumes calibration has already been performed and stored. In production systems, you must load stored calibration data and enable crosstalk compensation.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l3cx_api.h"

static void vl53lx_task(void *arg)
{
    VL53LX_Dev_t dev;
    VL53LX_DEV dev_handle = &dev;
    bool first_measurement = true;

    /* Set up the I2C address used by your sensor (0x52 is the default when the
     * XSHUT pin is pulled high). */
    dev.i2c_slave_address = 0x52;

    /* Initialise communication at 400 kHz. The ESP‑IDF I2C master driver
     * limits the clock to 400 kHz as stated in the documentation【462041596841165†L425-L447】. */
    if (VL53LX_CommsInitialise(dev_handle, 0, 400) != VL53LX_ERROR_NONE) {
        printf("Failed to initialise VL53LX comms\n");
        vTaskDelete(NULL);
    }

    /* Initialise the sensor. */
    if (VL53LX_WaitDeviceBooted(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_DataInit(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_SetDistanceMode(dev_handle, VL53LX_DISTANCEMODE_SHORT) != VL53LX_ERROR_NONE ||
        VL53LX_StartMeasurement(dev_handle) != VL53LX_ERROR_NONE) {
        printf("Failed to initialise VL53LX device\n");
        vTaskDelete(NULL);
    }

    /* Continuously read range data. */
    while (1) {
        VL53LX_MultiRangingData_t data;
        VL53LX_GetMultiRangingData(dev_handle, &data);
        if (data.NumberOfObjectsFound > 0) {
            printf("Distance: %u mm\n", data.RangeData[0].RangeMilliMeter);
        }
        /* Delay between measurements */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(vl53lx_task, "vl53lx", 4096, NULL, 5, NULL, 0);
}
```

## Enhanced Features for ESP32

### GPIO Control Functions
The platform layer now includes full GPIO control capabilities:

- **VL53LX_GpioXshutdown()**: Hardware reset control via XSHUT pin
- **VL53LX_GpioPowerEnable()**: Power control for sensor power management
- **VL53LX_GpioInterruptEnable()**: Interrupt-based measurement notifications
- **VL53LX_GpioSetMode()**, **VL53LX_GpioSetValue()**, **VL53LX_GpioGetValue()**: General GPIO operations

### ESP-IDF Logging Integration
VL53LX internal logging is integrated with ESP-IDF's logging system:

- Compatible with ESP-IDF log level control (menuconfig or esp_log_level_set())
- Consistent log formatting with other ESP-IDF components
- Enable via `-DVL53LX_LOG_ENABLE` build flag

### Examples Included
- **basic_usage**: Simple distance measurement
- **advanced_features**: Multi-object detection, GPIO control, device information
- **interrupt_based**: Efficient interrupt-driven measurements with FreeRTOS integration

## Directory structure

```
.
├── include/vl53lx/core   – Header files from the ST "BareDriver"
├── include/vl53lx/platform – Platform headers from the ST "BareDriver"
├── src/vl53lx/core       – ST driver sources
├── src/vl53lx/platform   – ESP‑IDF platform implementation
├── examples/               – Example projects demonstrating usage
├── library.json            – PlatformIO manifest
└── README.md               – This document
```

## Factory Calibration

**⚠️ MANDATORY FOR PRODUCTION**: The VL53LX sensor requires calibration to achieve optimal performance. This must be performed **once** during manufacturing.

### Calibration Sequence (EXACT ORDER REQUIRED)

```c
// 1. RefSPAD Calibration (no external target required)
VL53LX_Error status = VL53LX_PerformRefSpadManagement(&device);
if (status != VL53LX_ERROR_NONE) {
    // Handle calibration failure
    return false;
}

// 2. Crosstalk Calibration (600mm target in dark environment)
status = VL53LX_PerformXTalkCalibration(&device);
if (status != VL53LX_ERROR_NONE) {
    // Handle calibration failure
    return false;
}

// 3. Offset Calibration (known distance target, 2-80 MCps signal rate)
status = VL53LX_PerformOffsetPerVCSELCalibration(&device, 600); // 600mm target
if (status != VL53LX_ERROR_NONE) {
    // Handle calibration failure
    return false;
}

// 4. Set offset correction mode to match calibration method
VL53LX_SetOffsetCorrectionMode(&device, VL53LX_OFFSETCORRECTIONMODE_PERVCSEL);

// 5. Save calibration data to non-volatile storage
VL53LX_CalibrationData_t cal_data;
VL53LX_GetCalibrationData(&device, &cal_data);
save_calibration_to_storage(&cal_data);
```

### Critical Calibration Requirements

#### RefSPAD Calibration
- **Setup**: No target in front of sensor
- **Environment**: Normal ambient conditions
- **Purpose**: Optimize SPAD selection

#### Crosstalk Calibration  
- **Setup**: Target at exactly **600mm** distance
- **Environment**: **Dark environment** (no IR interference)
- **Target**: Any reflectance acceptable
- **⚠️ CRITICAL**: After loading calibration data, crosstalk compensation **MUST** be enabled:
  ```c
  VL53LX_SetXTalkCompensationEnable(&device, 1);
  ```

#### Offset Calibration
- **Setup**: Target at known distance (600mm recommended)
- **Environment**: Dark environment
- **Signal Rate**: **2-80 MCps** (CRITICAL REQUIREMENT)
- **Method**: Use `VL53LX_PerformOffsetPerVCSELCalibration()` for best results

### Startup Calibration Loading

```c
void load_and_apply_calibration(VL53LX_DEV device) {
    VL53LX_CalibrationData_t cal_data;
    
    // Load calibration from storage
    if (!load_calibration_from_storage(&cal_data)) {
        printf("ERROR: No calibration data found!\n");
        return;
    }
    
    // Apply calibration data
    VL53LX_SetCalibrationData(device, &cal_data);
    
    // CRITICAL: Enable crosstalk compensation
    VL53LX_SetXTalkCompensationEnable(device, 1);
    
    // Set offset correction mode
    VL53LX_SetOffsetCorrectionMode(device, VL53LX_OFFSETCORRECTIONMODE_PERVCSEL);
}
```

### Field Calibration (Repair Shops)

For devices where calibration data is lost:

```c
// Simplified zero-distance offset calibration
// Place target (e.g., paper) directly touching cover glass
VL53LX_PerformOffsetZeroDistanceCalibration(&device);
```

## Production Guidelines

### Timing Requirements
- **Boot + SW Standby + Init**: Exactly 40ms
- **First Valid Measurement**: 40ms + (2 × timing_budget)
- **Range1**: Must be discarded (no wrap-around check)
- **Range2**: First usable measurement

### Data Processing
```c
bool first_measurement = true;

while (ranging_active) {
    VL53LX_GetMultiRangingData(&device, &data);
    
    if (first_measurement) {
        // CRITICAL: Always discard Range1
        first_measurement = false;
        continue;
    }
    
    // Process valid data (Range2 onwards)
    process_measurement_data(&data);
}
```

### Error Handling
```c
VL53LX_Error status = VL53LX_GetMultiRangingData(&device, &data);
if (status != VL53LX_ERROR_NONE) {
    switch (status) {
        case VL53LX_ERROR_TIME_OUT:
            // Retry operation
            break;
        case VL53LX_ERROR_RANGE_ERROR:
            // Check interrupt handling
            break;
        case VL53LX_ERROR_CALIBRATION_WARNING:
            // Check/reload calibration data
            break;
        default:
            // Handle other errors
            break;
    }
}
```

## Licensing

The original VL53LX driver code is distributed by STMicroelectronics under a dual licence: GPL‑2.0 or BSD‑3‑Clause. This repository retains that licence. See the individual source files for details. Any modifications contained in the `platform` folder are provided under the same dual licence.