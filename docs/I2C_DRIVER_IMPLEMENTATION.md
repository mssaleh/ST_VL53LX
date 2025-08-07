# I2C Driver Implementation Details

This document explains the technical differences between the two I2C driver implementations supported by this library.

## Driver Architecture

### New I2C Master Driver (Default)
- **File:** `driver/i2c_master.h`
- **Available:** ESP-IDF v5.0+
- **Architecture:** Handle-based, object-oriented design
- **Memory:** Higher due to handle management
- **Performance:** Optimized for throughput and reliability

### Legacy I2C Driver (Backward Compatibility)
- **File:** `driver/i2c.h`
- **Available:** All ESP-IDF versions
- **Architecture:** Command-based, procedural design
- **Memory:** Lower memory footprint
- **Performance:** Stable and proven

## Implementation Differences

### Initialization

**New I2C Master Driver:**
```c
i2c_master_bus_config_t bus_config = { /* ... */ };
i2c_new_master_bus(&bus_config, &s_i2c_bus_handle);

i2c_device_config_t dev_cfg = { /* ... */ };
i2c_master_bus_add_device(s_i2c_bus_handle, &dev_cfg, &s_i2c_dev_handle);
```

**Legacy I2C Driver:**
```c
i2c_config_t conf = { /* ... */ };
i2c_param_config(I2C_PORT, &conf);
i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
```

### Write Operations

**New I2C Master Driver:**
```c
i2c_master_transmit(s_i2c_dev_handle, buffer, length, timeout);
```

**Legacy I2C Driver:**
```c
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, address | I2C_MASTER_WRITE, true);
i2c_master_write(cmd, buffer, length, true);
i2c_master_stop(cmd);
i2c_master_cmd_begin(I2C_PORT, cmd, timeout);
i2c_cmd_link_delete(cmd);
```

### Read Operations

**New I2C Master Driver:**
```c
i2c_master_transmit_receive(s_i2c_dev_handle, 
                           register_addr, 2,
                           read_buffer, read_length, 
                           timeout);
```

**Legacy I2C Driver:**
```c
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// Write register address
i2c_master_start(cmd);
i2c_master_write_byte(cmd, address | I2C_MASTER_WRITE, true);
i2c_master_write(cmd, register_addr, 2, true);
// Read data
i2c_master_start(cmd);  // Repeated start
i2c_master_write_byte(cmd, address | I2C_MASTER_READ, true);
i2c_master_read(cmd, read_buffer, read_length - 1, I2C_MASTER_ACK);
i2c_master_read_byte(cmd, &read_buffer[read_length - 1], I2C_MASTER_NACK);
i2c_master_stop(cmd);
i2c_master_cmd_begin(I2C_PORT, cmd, timeout);
i2c_cmd_link_delete(cmd);
```

## Address Handling

### Critical Difference: I2C Address Format

**New I2C Master Driver:**
- Expects **7-bit addresses** (clean device address)
- VL53LX API uses 8-bit addresses (includes R/W bit)
- **Conversion required:** `device_address = vl53lx_address >> 1`

**Legacy I2C Driver:**
- Uses **8-bit addresses** directly
- Compatible with VL53LX API address format
- **No conversion needed**

Example:
```c
// VL53LX API address: 0x52
#if VL53LX_I2C_DRIVER_LEGACY
    uint8_t i2c_addr = 0x52;  // Use directly
#else
    uint8_t i2c_addr = 0x52 >> 1;  // Convert to 0x29
#endif
```

## Error Handling

Both implementations map ESP-IDF errors to VL53LX error codes:
- `ESP_OK` → `VL53LX_ERROR_NONE`
- Other errors → `VL53LX_ERROR_CONTROL_INTERFACE`

## Performance Considerations

### New I2C Master Driver
- **Advantages:**
  - Better error reporting
  - Optimized for high-frequency operations
  - Built-in device management
  - Future-proof design

- **Disadvantages:**
  - Higher memory usage
  - Requires ESP-IDF v5.0+

### Legacy I2C Driver
- **Advantages:**
  - Lower memory footprint
  - Works with all ESP-IDF versions
  - Simple, proven design
  - Direct address compatibility

- **Disadvantages:**
  - More verbose API
  - Basic error reporting
  - Manual command management

## Selection Guidelines

### Choose New I2C Master Driver When:
- Using ESP-IDF v5.0 or later
- Memory is not critically constrained
- You want the latest features and optimizations
- Building new projects

### Choose Legacy I2C Driver When:
- Using older ESP-IDF versions (< v5.0)
- Memory usage is critical
- You have existing code using legacy I2C
- You need maximum backward compatibility

## Migration Path

To migrate from legacy to new driver:
1. Upgrade to ESP-IDF v5.0+
2. Remove `-DVL53LX_USE_LEGACY_I2C` from build flags
3. Rebuild the project
4. No code changes needed in application

To maintain legacy compatibility:
1. Keep `-DVL53LX_USE_LEGACY_I2C` in build flags
2. Works with any ESP-IDF version
3. Same VL53LX API usage

## Testing Both Implementations

The library has been designed to maintain identical VL53LX API behavior regardless of the underlying I2C driver implementation. Both drivers pass the same test suite and provide equivalent functionality.
