# Production Calibration Example

This example demonstrates the complete factory calibration procedure required for optimal VL53LX sensor performance, based on the latest VL53L3CX integration guide requirements.

## Overview

The VL53LX sensor requires **mandatory factory calibration** to achieve optimal performance. This calibration must be performed **once** during manufacturing and involves three critical steps that must be executed in exact order.

## Critical Calibration Requirements

### 1. RefSPAD Calibration
- **Setup**: No target in front of sensor
- **Environment**: Normal ambient conditions  
- **Purpose**: Optimize SPAD selection

### 2. Crosstalk Calibration
- **Setup**: Target at exactly **600mm** distance
- **Environment**: **Dark environment** (no IR interference)
- **Target**: Any reflectance acceptable
- **⚠️ CRITICAL**: Crosstalk compensation is disabled by default and must be explicitly enabled

### 3. Offset Calibration  
- **Setup**: Target at known distance (600mm recommended)
- **Environment**: Dark environment
- **Signal Rate**: **2-80 MCps** (CRITICAL REQUIREMENT)
- **Method**: Uses Per-VCSEL calibration for best results

## Key Features Demonstrated

### Factory Calibration Procedure
- Complete 3-step calibration sequence in correct order
- Proper error handling and warning interpretation
- NVS storage for calibration data persistence
- User prompts for proper setup between steps

### Production Initialization
- Automatic calibration data loading from storage
- **Critical**: Explicit crosstalk compensation enablement
- Proper offset correction mode configuration

### Range1 Discard Implementation
- **MANDATORY**: First measurement (Range1) must be discarded
- Range1 lacks wrap-around validation and is unreliable
- Demonstrates proper measurement loop with Range1 handling

### Comprehensive Error Handling
- Status code checking for all API calls
- Warning interpretation for calibration procedures
- Recovery strategies for common error conditions

## Hardware Setup

### Basic I2C Connection
```
ESP32-C6     VL53LX
--------     -------
GPIO8    <-> SDA
GPIO9    <-> SCL  
3.3V     <-> VDD
GND      <-> GND
```

### Advanced GPIO Control (Optional)
```
ESP32-C6     VL53LX
--------     -------
GPIO11   <-> XSHUT (reset)
GPIO10   <-- GPIO1 (interrupt)
GPIO12   --> Power control (optional)
```

## Calibration Setup Requirements

### RefSPAD Calibration
- Remove any targets from sensor field of view
- Normal room lighting acceptable
- Sensor should be uncovered

### Crosstalk Calibration  
- Place target at **exactly 600mm** from sensor
- **Dark environment** - turn off room lights
- Cover windows to prevent IR interference
- Any target material acceptable (paper, cardboard, etc.)

### Offset Calibration
- Keep target at **600mm** distance  
- Maintain **dark environment**
- Ensure target provides **2-80 MCps signal rate**
- Use matte white target for best signal strength

## Usage Instructions

1. **Build and Flash:**
   ```bash
   pio run -t upload -t monitor
   ```

2. **First Run (No Calibration Data):**
   - Example will detect missing calibration data
   - Automatic factory calibration procedure starts
   - Follow on-screen prompts for each calibration step
   - Calibration data automatically saved to NVS storage

3. **Subsequent Runs (With Calibration Data):**
   - Calibration data loaded automatically from storage
   - Crosstalk compensation enabled automatically
   - Measurement demonstration starts immediately

4. **Calibration Procedure:**
   - **Step 1**: RefSPAD - Press ENTER when sensor is uncovered
   - **Step 2**: Crosstalk - Setup 600mm target in dark, press ENTER
   - **Step 3**: Offset - Verify setup, press ENTER
   - Calibration data saved automatically

## Expected Output

### Factory Calibration
```
I (1234) VL53LX_PRODUCTION: Starting factory calibration procedure...
W (1235) VL53LX_PRODUCTION: Ensure proper setup for each calibration step!
I (1236) VL53LX_PRODUCTION: Step 1/3: RefSPAD calibration (ensure no target in front of sensor)
I (2345) VL53LX_PRODUCTION: RefSPAD calibration completed successfully
W (2346) VL53LX_PRODUCTION: Setup required for crosstalk calibration:
W (2347) VL53LX_PRODUCTION: - Place target at EXACTLY 600mm distance
W (2348) VL53LX_PRODUCTION: - Ensure DARK environment (no IR interference)
```

### Production Measurements
```
I (5678) VL53LX_PRODUCTION: Measurements started. First measurement (Range1) will be discarded.
W (5679) VL53LX_PRODUCTION: Range1 DISCARDED (StreamCount: 0) - no wrap-around check performed
I (5890) VL53LX_PRODUCTION: Measurement #1 (StreamCount: 1):
I (5891) VL53LX_PRODUCTION:   Objects found: 1
I (5892) VL53LX_PRODUCTION:   Target 0: 245 mm (Signal: 12.34 MCps, Sigma: 2.1 mm)
```

## Important Notes

### Production Requirements
- **Mandatory**: Perform calibration once during manufacturing
- **Critical**: Store calibration data in non-volatile memory  
- **Essential**: Enable crosstalk compensation in production code
- **Required**: Always discard Range1 measurement

### Error Handling
- Check all VL53LX function return values
- Handle calibration warnings appropriately
- Implement recovery strategies for communication errors
- Monitor signal rates during offset calibration

### Performance Optimization
- Use interrupt-based measurement for best performance
- Configure timing budget based on application requirements
- Monitor StreamCount for missed measurements detection
- Implement filtering for noisy environments

## Troubleshooting

### Calibration Issues
- **RefSPAD warnings**: Check ambient conditions, ensure no objects in FOV
- **Crosstalk failure**: Verify 600mm distance, ensure dark environment
- **Offset failure**: Check signal rate (must be 2-80 MCps), verify setup

### Measurement Issues  
- **No objects detected**: Check target distance and reflectance
- **Range status errors**: Verify calibration data loaded correctly
- **Inconsistent readings**: Ensure Range1 is properly discarded

### Communication Errors
- **I2C timeouts**: Check wiring, pull-up resistors, I2C frequency
- **Boot failures**: Verify power supply, XSHUT pin configuration
- **Address conflicts**: Ensure correct I2C address (0x52 default)

## Integration Notes

This example provides a complete template for production integration:

1. Copy calibration functions to your production code
2. Integrate NVS storage functions for your storage system  
3. Add calibration procedure to manufacturing test suite
4. Implement Range1 discard in your measurement loops
5. Enable crosstalk compensation in all production builds

The calibration data format is compatible with ST's calibration tools and can be transferred between different storage systems as needed.