# Legacy I2C Driver Example

This example demonstrates using the VL53LX library with the **legacy ESP-IDF I2C driver** for backward compatibility with older ESP-IDF versions.

## Key Features

- **Legacy I2C Driver:** Uses `driver/i2c.h` instead of the new `driver/i2c_master.h`
- **Backward Compatibility:** Works with ESP-IDF versions prior to v5.0
- **Same API:** Identical VL53LX API usage regardless of I2C driver selection

## Configuration

The critical build flag that enables legacy I2C support:

```ini
build_flags =
    -DVL53LX_USE_LEGACY_I2C     ; Enable legacy I2C driver support
```

## I2C Driver Comparison

| Feature | Legacy I2C Driver | New I2C Master Driver |
|---------|-------------------|----------------------|
| ESP-IDF Version | All versions | v5.0+ |
| Include Header | `driver/i2c.h` | `driver/i2c_master.h` |
| API Style | Command-based | Handle-based |
| Performance | Good | Better |
| Memory Usage | Lower | Higher |
| Error Handling | Basic | Enhanced |

## When to Use Legacy I2C Driver

- **Older ESP-IDF versions** (< v5.0)
- **Existing projects** that cannot upgrade ESP-IDF
- **Memory-constrained applications** 
- **Simple I2C communication** requirements

## Hardware Setup

Same hardware setup as other examples:

- Connect VL53LX **VDD** to **3.3V**
- Connect VL53LX **GND** to **GND**
- Connect VL53LX **SDA** to **GPIO 8**
- Connect VL53LX **SCL** to **GPIO 9**

## Build and Run

```bash
pio run -t upload && pio device monitor
```

## Expected Output

```
Starting VL53LX Legacy I2C Driver Example
This example uses the legacy ESP-IDF I2C driver for backward compatibility
To use the new I2C master driver, remove -DVL53LX_USE_LEGACY_I2C from build_flags

VL53LX Legacy I2C Driver Example
Using legacy I2C driver for ESP-IDF backward compatibility
Legacy I2C driver initialized successfully
VL53LX sensor initialized successfully
Discarding first measurement (Range1)
Distance: 245 mm (Status: 0, Signal: 1250 MCps)
Distance: 243 mm (Status: 0, Signal: 1275 MCps)
...
```

## Switching to New I2C Master Driver

To switch back to the new I2C master driver:

1. Remove `-DVL53LX_USE_LEGACY_I2C` from `build_flags`
2. Ensure ESP-IDF v5.0 or later
3. Rebuild the project

The VL53LX API usage remains identical - only the underlying I2C implementation changes.
