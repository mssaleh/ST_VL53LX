# VL53LX Basic Usage Example

This example demonstrates the basic functionality of the VL53LX time-of-flight sensor using the ESP-IDF framework.

## Features Demonstrated

- Basic sensor initialization and configuration
- Simple polling-based distance measurement
- Basic error handling and logging
- Single-object distance detection

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

## Configuration

Default I2C pins can be overridden in `platformio.ini`:

```ini
build_flags =
    -DVL53LX_I2C_SDA=5
    -DVL53LX_I2C_SCL=6
```

## Expected Output

```
I (123) VL53LX_Basic: VL53LX Basic Example Starting
I (456) VL53LX_Basic: VL53LX sensor initialized successfully
I (567) VL53LX_Basic: Distance: 245 mm (status: 0)
I (678) VL53LX_Basic: Distance: 243 mm (status: 0)
```

## Usage

1. Wire the sensor according to the pin mapping above
2. Build and upload the example to your ESP32-C6
3. Open the serial monitor at 115200 baud
4. Place objects in front of the sensor to see distance measurements

## Notes

- This example uses SHORT distance mode (up to 1.3m)
- Measurements are taken every 100ms
- Status code 0 indicates a valid measurement
- Range accuracy: ±1% typical, ±3% maximum
