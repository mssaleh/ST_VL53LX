# Hardware Setup Guide

This guide explains how to connect the VL53LX sensor to ESP32 development boards and configure the hardware for optimal performance.

## Table of Contents
- [Pin Connections](#pin-connections)
- [ESP32-C6 DevKitM-1 Setup](#esp32-c6-devkitm-1-setup)
- [Power Requirements](#power-requirements)
- [I2C Configuration](#i2c-configuration)
- [Interrupt Setup](#interrupt-setup)
- [Multiple Sensors](#multiple-sensors)
- [PCB Design Guidelines](#pcb-design-guidelines)
- [Mechanical Considerations](#mechanical-considerations)

## Pin Connections

### VL53LX Sensor Pinout
The VL53LX sensor is available in various breakout board formats. Most common pinout:

```
VL53LX Breakout Board:
┌─────────────────┐
│  1 - GPIO1 (INT)│  ← Interrupt output (optional)
│  2 - SCL        │  ← I2C clock
│  3 - XSHUT      │  ← Shutdown/reset (optional)
│  4 - SDA        │  ← I2C data
│  5 - VDD        │  ← Power supply (2.6V-3.5V)
│  6 - GND        │  ← Ground
│ 7,8,9,10 - NC   │  ← Not connected
└─────────────────┘
```

### Basic Connection (Minimal Setup)
For basic operation, only 4 connections are required:

| VL53LX Pin | ESP32-C6 Pin | Description |
|--------------|--------------|-------------|
| VDD          | 3V3          | Power supply |
| GND          | GND          | Ground |
| SDA          | GPIO8        | I2C data (configurable) |
| SCL          | GPIO9        | I2C clock (configurable) |

### Advanced Connection (Full Features)
For interrupt-based operation and software reset:

| VL53LX Pin | ESP32-C6 Pin | Description |
|--------------|--------------|-------------|
| VDD          | 3V3          | Power supply |
| GND          | GND          | Ground |
| SDA          | GPIO8        | I2C data (configurable) |
| SCL          | GPIO9        | I2C clock (configurable) |
| GPIO1 (INT)  | GPIO10       | Interrupt output (configurable) |
| XSHUT        | GPIO11       | Shutdown/reset control (configurable) |

## ESP32-C6 DevKitM-1 Setup

### Board Overview
The ESP32-C6-DevKitM-1 is a compact development board based on ESP32-C6-MINI-1 module.

### GPIO Availability
ESP32-C6 has different GPIO capabilities compared to other ESP32 variants:

**Available GPIOs for sensor connections:**
- GPIO0-GPIO7: General purpose I/O
- GPIO8-GPIO11: General purpose I/O (used for sensor)
- GPIO12-GPIO21: General purpose I/O
- GPIO22-GPIO23: General purpose I/O

**Reserved/Special GPIOs to avoid:**
- GPIO24-GPIO30: SPI flash/PSRAM
- GPIO18-GPIO19: USB Serial/JTAG (can be used but may conflict with debugging)

### Recommended Pin Assignment
```c
// Default configuration (can be overridden)
#define VL53LX_I2C_SDA     GPIO_NUM_8
#define VL53LX_I2C_SCL     GPIO_NUM_9
#define VL53LX_INT_PIN     GPIO_NUM_10
#define VL53LX_XSHUT_PIN   GPIO_NUM_11
```

### Wiring Diagram
```
ESP32-C6-DevKitM-1      VL53LX Sensor
┌─────────────────┐     ┌─────────────────┐
│              3V3│────▶│VDD              │
│              GND│────▶│GND              │
│         GPIO8   │◄───▶│SDA              │
│         GPIO9   │────▶│SCL              │
│         GPIO10  │◄────│GPIO1 (INT)      │
│         GPIO11  │────▶│XSHUT            │
└─────────────────┘     └─────────────────┘
```

## Power Requirements

### Voltage Specifications
- **Operating voltage:** 2.6V to 3.5V (3.3V recommended)
- **I/O voltage:** Compatible with 3.3V logic levels
- **Current consumption:**
  - Active measurement: ~20mA typical
  - Standby mode: ~5µA
  - Shutdown mode: ~1µA

### Power Supply Considerations
1. **Use clean 3.3V supply:** ESP32-C6 DevKit provides regulated 3.3V
2. **Decoupling:** Add 100nF ceramic capacitor close to sensor VDD pin
3. **Voltage stability:** Ensure minimal voltage ripple during operation
4. **Current capability:** Power supply should provide at least 50mA total

### Power Sequencing
For reliable operation following ST specifications:
1. Apply VDD first and ensure stable voltage (2.6V-3.5V)
2. Wait for voltage to stabilize (minimum 2ms)
3. Release XSHUT (if connected) by setting GPIO high or leave floating
4. Wait additional 2ms before I2C communication
5. Call `VL53LX_WaitDeviceBooted()` to ensure device is ready

**Critical Notes:**
- VDD must be within 2.6V-3.5V range (3.3V nominal)
- XSHUT can be left floating (internal pull-up) or controlled via GPIO
- Boot time after power-on or XSHUT release is typically <2ms
- Device draws ~1µA in shutdown, ~5µA in standby, ~20mA during measurement

## I2C Configuration

### Bus Parameters
The library configures I2C with these parameters:
```c
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,              // Default I2C port
    .scl_io_num = VL53LX_I2C_SCL,     // GPIO9 default
    .sda_io_num = VL53LX_I2C_SDA,     // GPIO8 default
    .glitch_ignore_cnt = 7,             // Noise filtering
    .flags.enable_internal_pullup = true // Internal pull-ups enabled
};
```

### Custom I2C Pins
To use different GPIO pins, define them at build time:
```ini
; In platformio.ini
build_flags = 
    -DVL53LX_I2C_SDA=5
    -DVL53LX_I2C_SCL=6
```

### Pull-up Resistors
The library enables internal pull-ups by default (45kΩ typical). For most applications, this is sufficient. External pull-ups may be needed for:
- Long wire connections (>30cm)
- Multiple devices on the bus
- High-speed operation
- Improved noise immunity

**External pull-up values:**
- Short connections (<10cm): 10kΩ
- Medium connections (10-30cm): 4.7kΩ
- Long connections (>30cm): 2.2kΩ

### I2C Address and Communication
**Default Address:** 0x52 (8-bit format including R/W bit) = 0x29 (7-bit format)
- The ST API expects 8-bit addressing format (0x52)
- Most I2C libraries use 7-bit format (0x29)  
- The VL53LX has a fixed I2C address that cannot be changed via software
- XSHUT pin state does not affect I2C address (fixed in hardware)
- Communication speed: Up to 400 kHz (Fast Mode I2C)

**Important:** The VL53LX I2C address cannot be changed via software. For multiple sensors, use:
- Multiple I2C buses, or  
- XSHUT pin multiplexing to enable sensors sequentially

**I2C Address Format Clarification:**
```c
// Correct for ST VL53LX API (8-bit format)
dev.i2c_slave_address = 0x52;

// This would be wrong for ST API (7-bit format)
// dev.i2c_slave_address = 0x29;  // DON'T USE

// When using other I2C libraries, you may need 7-bit format:
// i2c_master_write_to_device(I2C_NUM_0, 0x29, ...);  // 7-bit for ESP-IDF I2C
```

## Interrupt Setup

### Interrupt Pin (GPIO1)
The sensor's GPIO1 pin provides interrupt functionality:
- **Active state:** Low (pulls down when data ready)
- **Type:** Open-drain output
- **Timing:** Pulse width ~1ms typical

### ESP32 Configuration
```c
gpio_config_t int_conf = {
    .pin_bit_mask = (1ULL << VL53LX_INT_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,    // Required for open-drain
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE       // Trigger on falling edge
};
```

### Benefits of Interrupt Mode
- Reduced CPU usage
- Lower power consumption
- Precise timing
- Allows system to sleep between measurements

## Production Calibration Requirements

### Critical Calibration Steps
For production systems, proper calibration is essential for optimal performance:

1. **Reference SPAD Management** (One-time per device):
   ```c
   VL53LX_PerformRefSpadManagement(dev);
   ```
   - Must be performed once during manufacturing
   - No external setup required (sensor can be uncovered)
   - Results are stored in device internal memory

2. **Crosstalk Calibration** (Recommended):
   ```c
   VL53LX_CalibrationData_t cal_data;
   VL53LX_PerformXTalkCalibration(dev, &cal_data);
   ```
   - Cover sensor completely during calibration
   - Compensates for internal reflections and optical crosstalk
   - Store `cal_data` in non-volatile memory

3. **Offset Calibration** (Highly Recommended):
   ```c
   VL53LX_PerformOffsetCalibration(dev, 100, &cal_data); // 100mm target
   ```
   - Use flat, matte white target at known distance (100mm typical)
   - Target must cover entire sensor field of view  
   - Compensates for sensor-to-sensor variations

4. **Calibration Data Management**:
   ```c
   // After each VL53LX_DataInit(), restore calibration:
   VL53LX_DataInit(dev);
   VL53LX_SetOffsetCalibrationData(dev, &cal_data);
   VL53LX_SetXTalkCalibrationData(dev, &cal_data);
   ```

### Calibration Environment Requirements
- **Temperature**: Perform at expected operating temperature (±5°C)
- **Lighting**: Consistent ambient lighting conditions
- **Target surface**: Clean, flat, matte white (>90% reflectivity)
- **Target size**: Minimum 5cm x 5cm to cover sensor FOV
- **Stability**: Mechanical vibration <0.1mm during calibration

## Multiple Sensors

### Address Limitation
Since VL53LX has a fixed I2C address, multiple sensors require:
1. Separate I2C buses, or
2. XSHUT multiplexing

### XSHUT Multiplexing Method
Connect each sensor's XSHUT to a different GPIO:

```c
// Sensor 1: XSHUT to GPIO11
// Sensor 2: XSHUT to GPIO12
// Sensor 3: XSHUT to GPIO13

// Initialize sensors one by one
gpio_set_level(GPIO11, 0);  // Disable sensor 1
gpio_set_level(GPIO12, 0);  // Disable sensor 2
gpio_set_level(GPIO13, 1);  // Enable sensor 3 only

// Configure sensor 3, then enable others
```

### Dual I2C Bus Method
ESP32-C6 supports two I2C controllers:

```c
// Bus 0: Sensor 1
bus_config.i2c_port = I2C_NUM_0;
bus_config.scl_io_num = GPIO9;
bus_config.sda_io_num = GPIO8;

// Bus 1: Sensor 2  
bus_config.i2c_port = I2C_NUM_1;
bus_config.scl_io_num = GPIO7;
bus_config.sda_io_num = GPIO6;
```

## PCB Design Guidelines

### Layout Recommendations
1. **Keep I2C traces short:** Minimize trace length between ESP32 and sensor
2. **Use ground plane:** Solid ground plane under I2C traces
3. **Trace impedance:** 50Ω controlled impedance for longer traces
4. **Via minimization:** Avoid vias in I2C signal paths

### Component Placement
1. **Sensor placement:** Mount sensor at PCB edge for unobstructed field of view
2. **Decoupling capacitor:** Place 100nF cap within 5mm of sensor VDD pin
3. **Pull-up resistors:** Place close to ESP32 if using external pull-ups
4. **Crystal isolation:** Keep sensor away from ESP32 crystal oscillator

### Mechanical Considerations
1. **Field of view:** Ensure 27° cone is unobstructed
2. **Window material:** Use IR-transparent materials (avoid standard glass)
3. **Mounting:** Secure mounting to prevent vibration
4. **Environmental protection:** Consider IP rating requirements

### Signal Integrity
```
I2C Trace Specifications:
- Trace width: 0.1-0.2mm (4-8 mil)
- Spacing: >0.1mm (4 mil) from other traces
- Length matching: ±0.5mm between SDA/SCL
- Ground clearance: 0.1mm minimum
```

## Mechanical Considerations

### Sensor Orientation
- **Standard mounting:** Sensor facing forward, parallel to PCB
- **Field of view:** 27° full angle cone
- **Minimum distance:** 0mm (can measure very close objects)
- **Maximum distance:** Depends on distance mode and target reflectivity

### Environmental Factors
1. **Temperature range:** -40°C to +85°C (check specific part number)
2. **Humidity:** Non-condensing environments preferred
3. **Vibration:** Mount securely to avoid measurement errors
4. **Dust/contamination:** Cover sensor window if needed

### Window Materials
For applications requiring protective windows:
- **Recommended:** AR-coated glass, acrylic, polycarbonate
- **Avoid:** Standard window glass (absorbs IR)
- **Thickness:** <2mm preferred
- **Angle:** Mount perpendicular to avoid reflections

### Calibration Considerations
- **Target surface:** Use matte white surface for calibration
- **Distance:** Perform offset calibration at intended operating distance
- **Environment:** Calibrate in similar lighting conditions to operation
- **Temperature:** Consider temperature compensation for precision applications

## Calibration and Production Setup

### Factory Calibration Requirements
The VL53LX requires calibration for optimal performance in production environments:

1. **Reference SPAD Management:**
   ```c
   // Perform once per device - stores data in sensor NVM
   VL53LX_Error status = VL53LX_PerformRefSpadManagement(dev_handle);
   if (status != VL53LX_ERROR_NONE) {
       ESP_LOGE(TAG, "SPAD management failed: %d", status);
   }
   ```

2. **Offset Calibration:**
   ```c
   // Use known target distance (e.g., 100mm white paper)
   VL53LX_CalibrationData_t cal_data;
   int32_t target_distance_mm = 100;
   
   status = VL53LX_PerformOffsetCalibration(dev_handle, target_distance_mm, &cal_data);
   if (status == VL53LX_ERROR_NONE) {
       // Store cal_data in application NVM for later restoration
       save_calibration_data(&cal_data);
   }
   ```

3. **Crosstalk Calibration:**
   ```c
   // Perform with sensor covered (no target)
   status = VL53LX_PerformXTalkCalibration(dev_handle, &cal_data);
   ```

### Environmental Testing Guidelines

**Temperature Testing:**
- Operate across full temperature range (-40°C to +85°C)
- Verify performance at temperature extremes
- Document any temperature-dependent offset drift

**Vibration Testing:**
- Mount sensor securely to minimize mechanical vibration
- Test measurement stability under expected vibration conditions
- Consider mechanical damping for high-vibration environments

**EMI/EMC Considerations:**
- Keep sensor away from switching power supplies
- Use proper PCB ground planes
- Consider ferrite beads on power lines if EMI is present
- Test in actual electromagnetic environment

## Advanced Configuration Options

### Timing Budget Optimization
The measurement timing budget directly affects accuracy, range, and measurement rate:

```c
// Fast measurements (20ms) - reduced range and accuracy
VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 20000);

// Balanced performance (100ms) - good range and accuracy
VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 100000);

// Maximum performance (1000ms) - best range and accuracy
VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 1000000);
```

### Region of Interest (ROI) Configuration
Configure smaller ROI for improved precision on specific targets:

```c
VL53LX_UserRoi_t roi;
// Center 4x4 SPAD array (of 16x16 total)
roi.TopLeftX = 6;
roi.TopLeftY = 9;
roi.BotRightX = 9;
roi.BotRightY = 6;

VL53LX_SetUserROI(dev_handle, &roi);
```

### Limit Check Configuration
Set thresholds for automatic measurement validation:

```c
// Set sigma (uncertainty) limit
VL53LX_SetLimitCheckEnable(dev_handle, VL53LX_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
VL53LX_SetLimitCheckValue(dev_handle, VL53LX_CHECKENABLE_SIGMA_FINAL_RANGE, 
                          FixPoint1616_t(15 << 16)); // 15mm limit

// Set signal rate limit  
VL53LX_SetLimitCheckEnable(dev_handle, VL53LX_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
VL53LX_SetLimitCheckValue(dev_handle, VL53LX_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                          FixPoint1616_t(0.25 * (1 << 16))); // 0.25 MCPS minimum
```
