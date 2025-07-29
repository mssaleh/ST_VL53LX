/*
 * Advanced VL53LX example demonstrating multiple measurement modes,
 * multi-object detection, and advanced sensor features.
 * 
 * This example shows how to:
 * - Configure different distance modes (short, medium, long)
 * - Perform multi-object detection
 * - Use various measurement timing budgets
 * - Handle sensor calibration
 * - Read device information
 * 
 * Hardware setup:
 * - Connect VL53LX SDA to GPIO 8 (or override via VL53LX_I2C_SDA)
 * - Connect VL53LX SCL to GPIO 9 (or override via VL53LX_I2C_SCL)
 * - Connect VL53LX VDD to 3.3V
 * - Connect VL53LX GND to GND
 * - Connect VL53LX XSHUT to GPIO 11 (reset pin - controlled via GPIO functions)
 * - Optional: Connect VL53LX GPIO1 to GPIO 10 for interrupt handling
 * - Optional: Connect power control circuit to GPIO 12 (VL53LX_PWR_PIN)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "vl53l3cx_api.h"

static const char *TAG = "VL53LX_Advanced";

/* Sensor configuration */
#define SENSOR_I2C_ADDRESS  0x52
#define MEASUREMENT_PERIOD_MS 100

/* Function prototypes */
static void sensor_info_task(void *arg);
static void distance_measurement_task(void *arg);
static void print_device_info(VL53LX_DEV dev);
static void print_measurement_data(VL53LX_MultiRangingData_t *data);
static VL53LX_Error configure_sensor_advanced(VL53LX_DEV dev);

void app_main(void)
{
    ESP_LOGI(TAG, "Starting VL53LX Advanced Example");
    
    /* Create tasks for different sensor operations */
    xTaskCreatePinnedToCore(sensor_info_task, "sensor_info", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(distance_measurement_task, "distance_measure", 4096, NULL, 4, NULL, 1);
}

static void sensor_info_task(void *arg)
{
    VL53LX_Dev_t dev;
    VL53LX_DEV dev_handle = &dev;
    
    dev.i2c_slave_address = SENSOR_I2C_ADDRESS;
    
    ESP_LOGI(TAG, "Demonstrating advanced GPIO control features...");
    
    /* Demonstrate power control (if hardware supports it) */
    ESP_LOGI(TAG, "Powering on sensor via GPIO...");
    if (VL53LX_GpioPowerEnable(1) != VL53LX_ERROR_NONE) {
        ESP_LOGW(TAG, "Power control not available or failed");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Demonstrate sensor reset via XSHUT pin */
    ESP_LOGI(TAG, "Performing hardware reset via XSHUT pin...");
    VL53LX_GpioXshutdown(0);  // Reset (low)
    vTaskDelay(pdMS_TO_TICKS(10));
    VL53LX_GpioXshutdown(1);  // Release reset (high)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Initializing sensor communication...");
    
    if (VL53LX_CommsInitialise(dev_handle, 0, 400) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize I2C communication");
        vTaskDelete(NULL);
        return;
    }
    
    /* Wait for device to boot */
    if (VL53LX_WaitDeviceBooted(dev_handle) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Device failed to boot");
        vTaskDelete(NULL);
        return;
    }
    
    /* Print device information */
    print_device_info(dev_handle);
    
    /* This task runs once and then deletes itself */
    vTaskDelete(NULL);
}

static void distance_measurement_task(void *arg)
{
    VL53LX_Dev_t dev;
    VL53LX_DEV dev_handle = &dev;
    
    dev.i2c_slave_address = SENSOR_I2C_ADDRESS;
    
    /* Wait a bit for the info task to complete */
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Configuring sensor for advanced measurements...");
    
    /* Initialize and configure the sensor */
    if (configure_sensor_advanced(dev_handle) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to configure sensor");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting continuous measurements...");
    
    /* Start measurements */
    if (VL53LX_StartMeasurement(dev_handle) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to start measurements");
        vTaskDelete(NULL);
        return;
    }
    
    /* Measurement loop */
    while (1) {
        VL53LX_MultiRangingData_t data;
        uint8_t data_ready = 0;
        
        /* Check if new data is available */
        VL53LX_GetMeasurementDataReady(dev_handle, &data_ready);
        
        if (data_ready) {
            /* Get the measurement data */
            if (VL53LX_GetMultiRangingData(dev_handle, &data) == VL53LX_ERROR_NONE) {
                print_measurement_data(&data);
                
                /* Clear the interrupt */
                VL53LX_ClearInterruptAndStartMeasurement(dev_handle);
            }
        }
        
        /* Small delay to prevent overwhelming the system */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void print_device_info(VL53LX_DEV dev)
{
    VL53LX_DeviceInfo_t device_info;
    VL53LX_Error status;
    
    ESP_LOGI(TAG, "=== Device Information ===");
    
    status = VL53LX_GetDeviceInfo(dev, &device_info);
    if (status == VL53LX_ERROR_NONE) {
        ESP_LOGI(TAG, "Product Type: 0x%02X", device_info.ProductType);
        ESP_LOGI(TAG, "Product Revision: %d.%d", 
                 device_info.ProductRevisionMajor, 
                 device_info.ProductRevisionMinor);
    } else {
        ESP_LOGE(TAG, "Failed to get device info (error: %d)", status);
    }
    
    /* Read additional sensor information */
    uint16_t device_id;
    if (VL53LX_RdWord(dev, VL53LX_IDENTIFICATION__MODEL_ID, &device_id) == VL53LX_ERROR_NONE) {
        ESP_LOGI(TAG, "Model ID: 0x%04X (expected: 0xEACC)", device_id);
    }
    
    uint8_t revision_id;
    if (VL53LX_RdByte(dev, VL53LX_IDENTIFICATION__REVISION_ID, &revision_id) == VL53LX_ERROR_NONE) {
        ESP_LOGI(TAG, "Revision ID: 0x%02X", revision_id);
    }
    
    ESP_LOGI(TAG, "==========================");
}

static VL53LX_Error configure_sensor_advanced(VL53LX_DEV dev)
{
    VL53LX_Error status;
    
    /* Initialize sensor data structures */
    status = VL53LX_DataInit(dev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX_DataInit failed (error: %d)", status);
        return status;
    }
    
    /* Set distance mode - try different modes for different applications:
     * VL53LX_DISTANCEMODE_SHORT: Up to 1.3m, better ambient immunity
     * VL53LX_DISTANCEMODE_MEDIUM: Up to 3m, balanced performance  
     * VL53LX_DISTANCEMODE_LONG: Up to 4m, best range performance
     */
    status = VL53LX_SetDistanceMode(dev, VL53LX_DISTANCEMODE_MEDIUM);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX_SetDistanceMode failed (error: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "Distance mode set to MEDIUM");
    
    /* Set measurement timing budget in microseconds
     * Longer timing budgets provide better accuracy and maximum range
     * Shorter timing budgets enable faster measurements
     * Valid range: 20000 to 1000000 microseconds
     */
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(dev, 100000); // 100ms
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX_SetMeasurementTimingBudgetMicroSeconds failed (error: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "Measurement timing budget set to 100ms");
    
    /* Set inter-measurement period (time between measurements) */
    status = VL53LX_SetInterMeasurementPeriodMilliSeconds(dev, MEASUREMENT_PERIOD_MS);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53LX_SetInterMeasurementPeriodMilliSeconds failed (error: %d)", status);
        return status;
    }
    ESP_LOGI(TAG, "Inter-measurement period set to %dms", MEASUREMENT_PERIOD_MS);
    
    /* Configure ROI (Region of Interest) - optional
     * This defines the area of the sensor's field of view to use for measurements
     * Smaller ROIs can provide better precision for specific targets
     */
    VL53LX_UserRoi_t roi;
    roi.TopLeftX = 6;
    roi.TopLeftY = 9;  
    roi.BotRightX = 9;
    roi.BotRightY = 6;
    
    status = VL53LX_SetUserROI(dev, &roi);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGW(TAG, "VL53LX_SetUserROI failed (error: %d), using default ROI", status);
    } else {
        ESP_LOGI(TAG, "Custom ROI configured");
    }
    
    ESP_LOGI(TAG, "Advanced sensor configuration completed");
    return VL53LX_ERROR_NONE;
}

static void print_measurement_data(VL53LX_MultiRangingData_t *data)
{
    static int measurement_count = 0;
    measurement_count++;
    
    ESP_LOGI(TAG, "=== Measurement #%d ===", measurement_count);
    ESP_LOGI(TAG, "Stream Count: %u", data->StreamCount);
    ESP_LOGI(TAG, "Objects Found: %u", data->NumberOfObjectsFound);
    
    if (data->NumberOfObjectsFound == 0) {
        ESP_LOGI(TAG, "No objects detected");
        return;
    }
    
    /* Print information for each detected object */
    for (int i = 0; i < data->NumberOfObjectsFound && i < VL53LX_MAX_RANGE_RESULTS; i++) {
        VL53LX_TargetRangeData_t *obj = &data->RangeData[i];
        
        ESP_LOGI(TAG, "Object %d:", i + 1);
        ESP_LOGI(TAG, "  Range: %u mm", obj->RangeMilliMeter);
        ESP_LOGI(TAG, "  Status: %u", obj->RangeStatus);
        ESP_LOGI(TAG, "  Signal Rate: %u MCPS", obj->SignalRateRtnMegaCps);
        ESP_LOGI(TAG, "  Ambient Rate: %u MCPS", obj->AmbientRateRtnMegaCps);
        ESP_LOGI(TAG, "  Sigma: %u mm", obj->SigmaMilliMeter);
        
        /* Decode range status based on ST official documentation */
        const char *status_string;
        switch (obj->RangeStatus) {
            case VL53LX_RANGESTATUS_RANGE_VALID:
                status_string = "Valid";
                break;
            case VL53LX_RANGESTATUS_SIGMA_FAIL:
                status_string = "Sigma Fail (use VL53LX_SetLimitCheckEnable to adjust)";
                break;
            case VL53LX_RANGESTATUS_SIGNAL_FAIL:
                status_string = "Signal Fail (no target or range ignore threshold exceeded)";
                break;
            case VL53LX_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED:
                status_string = "Valid (Min Range Clipped)";
                break;
            case VL53LX_RANGESTATUS_OUTOFBOUNDS_FAIL:
                status_string = "Phase Out of Valid Limits";
                break;
            case VL53LX_RANGESTATUS_HARDWARE_FAIL:
                status_string = "Hardware Failure";
                break;
            case VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
                status_string = "Valid (No Wrap-Around Check)";
                break;
            case VL53LX_RANGESTATUS_WRAP_TARGET_FAIL:
                status_string = "Wrapped Target (no matching phase)";
                break;
            case VL53LX_RANGESTATUS_PROCESSING_FAIL:
                status_string = "Processing Failure";
                break;
            case VL53LX_RANGESTATUS_XTALK_SIGNAL_FAIL:
                status_string = "Crosstalk Signal Failure";
                break;
            case VL53LX_RANGESTATUS_SYNCRONISATION_INT:
                status_string = "First Interrupt (ignore data)";
                break;
            case VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
                status_string = "Valid (Merged Pulses)";
                break;
            case VL53LX_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
                status_string = "Target Present (Insufficient Signal)";
                break;
            case VL53LX_RANGESTATUS_MIN_RANGE_FAIL:
                status_string = "Minimum Range Failure";
                break;
            case VL53LX_RANGESTATUS_RANGE_INVALID:
                status_string = "Invalid Range";
                break;
            case VL53LX_RANGESTATUS_NONE:
                status_string = "No Update (no meaningful data)";
                break;
            default:
                status_string = "Unknown Status";
                break;
        }
        ESP_LOGI(TAG, "  Status Description: %s", status_string);
    }
    
    ESP_LOGI(TAG, "========================");
}
