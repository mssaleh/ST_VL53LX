# VL53LX Interrupt-Based Example

This example demonstrates efficient interrupt-based operation of the VL53LX sensor, allowing the CPU to perform other tasks while waiting for measurements.

## Features Demonstrated

- Interrupt-based measurement (no polling required)
- GPIO interrupt handling with FreeRTOS synchronization
- Hardware reset control via XSHUT pin
- Multi-task architecture with measurement processing
- Timeout handling and error recovery
- Queue-based event processing
- Production-ready interrupt service routine (ISR)

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
| GPIO1 (INT)  | GPIO10       | Interrupt output |
| XSHUT        | GPIO11       | Shutdown/reset control |

## Pin Configuration

Default pins can be overridden in the source code:

```c
#define VL53LX_INT_PIN    GPIO_NUM_10
#define VL53LX_XSHUT_PIN  GPIO_NUM_11
```

I2C pins can be configured in `platformio.ini`:

```ini
build_flags =
    -DVL53LX_I2C_SDA=8
    -DVL53LX_I2C_SCL=9
```

## Interrupt Operation

### How It Works

1. **Sensor Configuration**: Sensor is configured for continuous measurements
2. **Interrupt Setup**: GPIO10 configured for falling edge interrupts
3. **Measurement Cycle**: 
   - Sensor starts measurement automatically
   - When data is ready, sensor pulls GPIO1 low
   - ESP32 interrupt service routine (ISR) triggers
   - ISR sends event to FreeRTOS queue and releases semaphore
   - Main task processes measurement data
   - Cycle repeats automatically

### Advantages

- **CPU Efficiency**: No polling required - CPU free for other tasks
- **Low Power**: System can sleep between measurements
- **Precise Timing**: Immediate response to sensor data ready
- **Scalability**: Easy to add other sensors or tasks

## Expected Output

```
I (123) VL53LX_Interrupt: Starting VL53LX Interrupt-based Example
I (234) VL53LX_Interrupt: Interrupt GPIO setup complete on pin 10
I (345) VL53LX_Interrupt: Reset GPIO setup complete on pin 11
I (456) VL53LX_Interrupt: Tasks created successfully
I (567) VL53LX_Interrupt: Sensor task started
I (678) VL53LX_Interrupt: Sensor reset completed
I (789) VL53LX_Interrupt: Sensor initialization completed successfully
I (890) VL53LX_Interrupt: Starting interrupt-based measurements...
I (901) VL53LX_Interrupt: Measurement processor task started
I (1012) VL53LX_Interrupt: Event #1: timestamp=1000ms, interval=0ms
I (1123) VL53LX_Interrupt: Measurement #1: 1 objects detected
I (1134) VL53LX_Interrupt: Primary object: 245 mm (status: 0)
```

## Task Architecture

### Sensor Task (Core 0)
- Handles I2C communication with sensor
- Processes interrupts and reads measurement data
- Manages sensor state and error recovery
- Priority: 5 (high)

### Measurement Processor Task (Core 1)
- Processes measurement events from queue
- Calculates timing intervals and statistics
- Can be extended for data logging, filtering, etc.
- Priority: 4 (normal)

## Error Handling

The example includes comprehensive error handling:

- **Communication Timeouts**: 5-second timeout for measurements
- **Sensor Responsiveness**: Automatic health checks
- **Interrupt Validation**: Verifies data ready before processing
- **Hardware Reset**: Software-controlled sensor reset via XSHUT
- **Task Recovery**: Graceful task termination on critical errors

## Performance Characteristics

- **Measurement Rate**: Up to 20 Hz (configurable)
- **CPU Overhead**: Minimal - only during actual measurements
- **Memory Usage**: ~8KB total for both tasks
- **Interrupt Latency**: <100µs typical response time

## Usage

1. Wire the sensor according to the pin mapping above (including interrupt pin)
2. Build and upload the example to your ESP32-C6
3. Open the serial monitor at 115200 baud
4. Observe interrupt-driven measurements with timing information
5. Place objects in front of sensor to see measurements

## Customization

### Measurement Rate
Adjust inter-measurement period:
```c
VL53LX_SetInterMeasurementPeriodMilliSeconds(g_sensor_handle, 100); // 100ms = 10 Hz
```

### Timing Budget
Modify measurement accuracy vs. speed:
```c
VL53LX_SetMeasurementTimingBudgetMicroSeconds(g_sensor_handle, 50000); // 50ms
```

### Processing Tasks
Extend the measurement processor task for:
- Data filtering and averaging
- Wireless transmission
- Data logging to storage
- Multi-sensor fusion

## Production Considerations

This example demonstrates production-ready techniques:

- **ISR Safety**: Minimal ISR execution time
- **Thread Safety**: Proper FreeRTOS synchronization
- **Error Recovery**: Comprehensive error handling
- **Resource Management**: Proper cleanup and task management
- **Hardware Control**: Software reset capability

For production deployment:
- Add calibration procedures
- Implement data persistence
- Add communication protocols (WiFi, Bluetooth)
- Consider power management features
