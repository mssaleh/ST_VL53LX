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

    /* Initialize communication at 400 kHz using the LEGACY I2C driver.
     * This example demonstrates backward compatibility with older ESP-IDF versions
     * by using the legacy I2C driver (driver/i2c.h) instead of the new 
     * I2C master driver (driver/i2c_master.h).
     */
    printf("VL53LX Legacy I2C Driver Example\n");
    printf("Using legacy I2C driver for ESP-IDF backward compatibility\n");
    
    if (VL53LX_CommsInitialise(dev_handle, 0, 400) != VL53LX_ERROR_NONE) {
        printf("Failed to initialise VL53LX comms with legacy I2C driver\n");
        vTaskDelete(NULL);
    }
    printf("Legacy I2C driver initialized successfully\n");

    /* Initialize the sensor. */
    if (VL53LX_WaitDeviceBooted(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_DataInit(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_SetDistanceMode(dev_handle, VL53LX_DISTANCEMODE_SHORT) != VL53LX_ERROR_NONE ||
        VL53LX_StartMeasurement(dev_handle) != VL53LX_ERROR_NONE) {
        printf("Failed to initialise VL53LX device\n");
        vTaskDelete(NULL);
    }
    printf("VL53LX sensor initialized successfully\n");

    /* Continuously read range data. */
    while (1) {
        VL53LX_MultiRangingData_t data;
        VL53LX_Error status = VL53LX_GetMultiRangingData(dev_handle, &data);
        
        if (status == VL53LX_ERROR_NONE) {
            if (first_measurement) {
                /* CRITICAL: Always discard Range1 (first measurement) */
                printf("Discarding first measurement (Range1)\n");
                first_measurement = false;
            } else {
                /* Process valid data (Range2 onwards) */
                if (data.NumberOfObjectsFound > 0) {
                    printf("Distance: %u mm (Status: %u, Signal: %u MCps)\n", 
                           data.RangeData[0].RangeMilliMeter,
                           data.RangeData[0].RangeStatus,
                           data.RangeData[0].SignalRateRtnMegaCps);
                } else {
                    printf("No objects detected\n");
                }
            }
        } else {
            printf("Error reading measurement: %d\n", status);
        }
        
        /* Delay between measurements */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    printf("Starting VL53LX Legacy I2C Driver Example\n");
    printf("This example uses the legacy ESP-IDF I2C driver for backward compatibility\n");
    printf("To use the new I2C master driver, remove -DVL53LX_USE_LEGACY_I2C from build_flags\n\n");
    
    xTaskCreatePinnedToCore(vl53lx_task, "vl53lx", 4096, NULL, 5, NULL, 0);
}
