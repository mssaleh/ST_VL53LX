# VL53LX Advanced Features Example

This example demonstrates advanced features of the VL53LX sensor including multi-object detection, device information reading, and comprehensive range status handling.

## Features Demonstrated

- Complete sensor initialization with advanced configuration
- Device information retrieval and display
- Multi-object detection (up to 4 objects simultaneously)
- Multiple distance modes (SHORT, MEDIUM, LONG)
- Configurable measurement timing budgets
- Region of Interest (ROI) configuration
- Comprehensive range status interpretation
- Production-ready error handling

## Hardware Requirements

- ESP32-C6 development board (or compatible ESP32 variant)
- VL53LX breakout board or module
- Connecting wires

## Wiring

| VL53LX Pin | ESP32-C6 Pin | Description |
|--------------|--------------|-------------|
| VDD          | 3V3          | Power supply |
| GND          | GND          | Ground |
| SDA          | GPIO8        | I2C data (configurable) |
| SCL          | GPIO9        | I2C clock (configurable) |

## Configuration Options

### I2C Pin Configuration
```ini
build_flags =
    -DVL53LX_I2C_SDA=5
    -DVL53LX_I2C_SCL=6
```

### Distance Modes
- **SHORT**: Up to 1.3m, better ambient immunity, fastest measurements
- **MEDIUM**: Up to 3m, balanced performance (used in this example)
- **LONG**: Up to 4m, best range performance, slower measurements

### Timing Budget Options
- 50ms: Fast measurements, reduced accuracy
- 100ms: Good balance (used in this example)
- 200ms: High accuracy, slower update rate

## Expected Output

```
I (123) VL53LX_Advanced: Starting VL53LX Advanced Example
I (234) VL53LX_Advanced: === Device Information ===
I (345) VL53LX_Advanced: Product Type: 0xAA
I (456) VL53LX_Advanced: Product Revision: 1.0
I (567) VL53LX_Advanced: Model ID: 0xEACC (expected: 0xEACC)
I (678) VL53LX_Advanced: Distance mode set to MEDIUM
I (789) VL53LX_Advanced: === Measurement #1 ===
I (890) VL53LX_Advanced: Objects Found: 2
I (901) VL53LX_Advanced: Object 1:
I (912) VL53LX_Advanced:   Range: 245 mm
I (923) VL53LX_Advanced:   Status: 0
I (934) VL53LX_Advanced:   Status Description: Valid
```

## Multi-Object Detection

The example can detect multiple objects simultaneously when they are:
- Separated by at least 5cm
- Within the sensor's field of view (27° cone)
- Have sufficient signal strength

## Range Status Codes

The example provides detailed interpretation of all range status codes:

- **Status 0**: Valid measurement
- **Status 1**: Sigma fail (measurement uncertainty too high)
- **Status 2**: Signal fail (no target or insufficient signal)
- **Status 4**: Out of bounds (phase limits exceeded)
- **Status 5**: Hardware failure
- And many more...

## Usage

1. Wire the sensor according to the pin mapping above
2. Build and upload the example to your ESP32-C6
3. Open the serial monitor at 115200 baud
4. Place single or multiple objects in front of the sensor
5. Observe detailed measurement information and status interpretation

## Advanced Configuration

The example demonstrates:
- Custom ROI (Region of Interest) configuration
- Measurement timing budget adjustment
- Inter-measurement period configuration
- Device information reading from registers

## Production Notes

This example includes production-ready features:
- Comprehensive error handling
- Range status interpretation
- Device validation
- Performance optimization settings

For production use, consider implementing:
- Calibration procedures (offset, crosstalk)
- Temperature compensation
- Data filtering and averaging
- Interrupt-based operation (see interrupt_based example)
