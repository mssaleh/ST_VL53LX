/*
 * Production Calibration Example for VL53LX Sensor
 * 
 * This example demonstrates the complete factory calibration procedure
 * required for optimal VL53LX sensor performance. This calibration must
 * be performed once during manufacturing and the results stored in
 * non-volatile memory.
 * 
 * CRITICAL REQUIREMENTS:
 * 1. RefSPAD calibration (no target)
 * 2. Crosstalk calibration (600mm target in dark environment)
 * 3. Offset calibration (600mm target, 2-80 MCps signal rate)
 * 4. Enable crosstalk compensation after loading calibration data
 * 5. Discard Range1 measurement (no wrap-around validation)
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "vl53l3cx_api.h"

static const char *TAG = "VL53LX_PRODUCTION";

// NVS storage key for calibration data
#define CALIBRATION_NVS_KEY "vl53lx_cal"
#define CALIBRATION_NAMESPACE "vl53lx"

// Calibration state tracking
typedef struct {
    bool refspad_done;
    bool crosstalk_done;
    bool offset_done;
    uint32_t timestamp;
} calibration_status_t;

// Function prototypes
static bool save_calibration_to_storage(const VL53LX_CalibrationData_t *cal_data);
static bool load_calibration_from_storage(VL53LX_CalibrationData_t *cal_data);
static bool perform_factory_calibration(VL53LX_Dev_t *pdev);
static void production_sensor_init(VL53LX_Dev_t *pdev);
static void demonstrate_measurement_with_range1_discard(VL53LX_Dev_t *pdev);

/**
 * Save calibration data to NVS storage
 */
static bool save_calibration_to_storage(const VL53LX_CalibrationData_t *cal_data)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(CALIBRATION_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_blob(nvs_handle, CALIBRATION_NVS_KEY, cal_data, sizeof(VL53LX_CalibrationData_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration data: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration data saved successfully");
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to commit calibration data: %s", esp_err_to_name(err));
        return false;
    }
}

/**
 * Load calibration data from NVS storage
 */
static bool load_calibration_from_storage(VL53LX_CalibrationData_t *cal_data)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t required_size = sizeof(VL53LX_CalibrationData_t);
    
    err = nvs_open(CALIBRATION_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration data found in storage");
        return false;
    }
    
    err = nvs_get_blob(nvs_handle, CALIBRATION_NVS_KEY, cal_data, &required_size);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK && required_size == sizeof(VL53LX_CalibrationData_t)) {
        ESP_LOGI(TAG, "Calibration data loaded successfully");
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to load calibration data: %s", esp_err_to_name(err));
        return false;
    }
}

/**
 * Complete factory calibration procedure
 * CRITICAL: Must be performed in exact order specified by ST guide
 */
static bool perform_factory_calibration(VL53LX_Dev_t *pdev)
{
    VL53LX_Error status;
    calibration_status_t cal_status = {0};
    
    ESP_LOGI(TAG, "Starting factory calibration procedure...");
    ESP_LOGW(TAG, "Ensure proper setup for each calibration step!");
    
    // STEP 1: RefSPAD Calibration
    // Setup: No target in front of sensor, normal ambient conditions
    ESP_LOGI(TAG, "Step 1/3: RefSPAD calibration (ensure no target in front of sensor)");
    
    status = VL53LX_PerformRefSpadManagement(pdev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "RefSPAD calibration failed: %d", status);
        return false;
    }
    
    // Handle RefSPAD warnings
    if (status == VL53LX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS) {
        ESP_LOGW(TAG, "RefSPAD: Insufficient SPADs available (<5)");
    } else if (status == VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH) {
        ESP_LOGW(TAG, "RefSPAD: Reference rate too high (>40 Mcps), offset stability may be degraded");
    } else if (status == VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW) {
        ESP_LOGW(TAG, "RefSPAD: Reference rate too low (<10 Mcps), offset stability may be degraded");
    }
    
    cal_status.refspad_done = true;
    ESP_LOGI(TAG, "RefSPAD calibration completed successfully");
    
    // Wait for user to setup crosstalk calibration
    ESP_LOGW(TAG, "Setup required for crosstalk calibration:");
    ESP_LOGW(TAG, "- Place target at EXACTLY 600mm distance");
    ESP_LOGW(TAG, "- Ensure DARK environment (no IR interference)");
    ESP_LOGW(TAG, "- Any target reflectance acceptable");
    ESP_LOGW(TAG, "Press ENTER when ready for crosstalk calibration...");
    getchar(); // Wait for user input
    
    // STEP 2: Crosstalk Calibration
    // Setup: 600mm target in dark environment
    ESP_LOGI(TAG, "Step 2/3: Crosstalk calibration (600mm target, dark environment)");
    
    status = VL53LX_PerformXTalkCalibration(pdev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Crosstalk calibration failed: %d", status);
        return false;
    }
    
    // Handle crosstalk warnings
    if (status == VL53LX_WARNING_XTALK_MISSING_SAMPLES) {
        ESP_LOGW(TAG, "Crosstalk: Missing samples warning - check setup");
    }
    
    cal_status.crosstalk_done = true;
    ESP_LOGI(TAG, "Crosstalk calibration completed successfully");
    
    // Wait for user to setup offset calibration
    ESP_LOGW(TAG, "Setup required for offset calibration:");
    ESP_LOGW(TAG, "- Target at known distance (600mm recommended)");
    ESP_LOGW(TAG, "- Signal rate between 2-80 MCps (CRITICAL!)");
    ESP_LOGW(TAG, "- Dark environment (no IR interference)");
    ESP_LOGW(TAG, "Press ENTER when ready for offset calibration...");
    getchar(); // Wait for user input
    
    // STEP 3: Offset Calibration
    // Setup: Known distance target with 2-80 MCps signal rate
    ESP_LOGI(TAG, "Step 3/3: Offset calibration (600mm target, 2-80 MCps signal)");
    
    // Use Per-VCSEL calibration for best results
    status = VL53LX_PerformOffsetPerVCSELCalibration(pdev, 600);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Offset calibration failed: %d", status);
        return false;
    }
    
    // Handle offset warnings
    if (status == VL53LX_WARNING_OFFSET_CAL_INSUFFICIENT_MM1_SPADS) {
        ESP_LOGW(TAG, "Offset: Signal too low - accuracy may be degraded");
    } else if (status == VL53LX_WARNING_OFFSET_CAL_PRE_RANGE_RATE_TOO_HIGH) {
        ESP_LOGW(TAG, "Offset: Signal too high - accuracy may be degraded");
    }
    
    cal_status.offset_done = true;
    ESP_LOGI(TAG, "Offset calibration completed successfully");
    
    // STEP 4: Set offset correction mode to match calibration method
    status = VL53LX_SetOffsetCorrectionMode(pdev, VL53LX_OFFSETCORRECTIONMODE_PERVCSEL);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set offset correction mode: %d", status);
        return false;
    }
    
    // STEP 5: Save calibration data to non-volatile storage
    VL53LX_CalibrationData_t cal_data;
    status = VL53LX_GetCalibrationData(pdev, &cal_data);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to retrieve calibration data: %d", status);
        return false;
    }
    
    if (!save_calibration_to_storage(&cal_data)) {
        ESP_LOGE(TAG, "Failed to save calibration data to storage");
        return false;
    }
    
    cal_status.timestamp = (uint32_t)(esp_timer_get_time() / 1000000); // seconds since boot
    
    ESP_LOGI(TAG, "Factory calibration completed successfully!");
    ESP_LOGI(TAG, "RefSPAD: %s", cal_status.refspad_done ? "DONE" : "FAILED");
    ESP_LOGI(TAG, "Crosstalk: %s", cal_status.crosstalk_done ? "DONE" : "FAILED");
    ESP_LOGI(TAG, "Offset: %s", cal_status.offset_done ? "DONE" : "FAILED");
    
    return true;
}

/**
 * Production sensor initialization with calibration loading
 */
static void production_sensor_init(VL53LX_Dev_t *pdev)
{
    VL53LX_Error status;
    
    ESP_LOGI(TAG, "Initializing sensor for production use...");
    
    // Initialize communication
    if (VL53LX_CommsInitialise(pdev, 0, 400) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize communications");
        return;
    }
    
    // Wait for device boot - exactly 40ms required
    if (VL53LX_WaitDeviceBooted(pdev) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to wait for device boot");
        return;
    }
    
    // Initialize sensor data structures
    if (VL53LX_DataInit(pdev) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize sensor data");
        return;
    }
    
    // Load calibration data from storage
    VL53LX_CalibrationData_t cal_data;
    if (!load_calibration_from_storage(&cal_data)) {
        ESP_LOGE(TAG, "No calibration data available! Must perform factory calibration first.");
        return;
    }
    
    // Apply calibration data
    status = VL53LX_SetCalibrationData(pdev, &cal_data);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set calibration data: %d", status);
        return;
    }
    
    // CRITICAL: Enable crosstalk compensation (disabled by default!)
    status = VL53LX_SetXTalkCompensationEnable(pdev, 1);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to enable crosstalk compensation: %d", status);
        return;
    }
    
    // Set offset correction mode (must match calibration method)
    status = VL53LX_SetOffsetCorrectionMode(pdev, VL53LX_OFFSETCORRECTIONMODE_PERVCSEL);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set offset correction mode: %d", status);
        return;
    }
    
    ESP_LOGI(TAG, "Production sensor initialization completed successfully");
    ESP_LOGI(TAG, "Crosstalk compensation: ENABLED");
    ESP_LOGI(TAG, "Offset correction mode: PER-VCSEL");
}

/**
 * Demonstrate proper measurement handling with Range1 discard
 */
static void demonstrate_measurement_with_range1_discard(VL53LX_Dev_t *pdev)
{
    VL53LX_Error status;
    VL53LX_MultiRangingData_t data;
    bool first_measurement = true;
    int measurement_count = 0;
    
    ESP_LOGI(TAG, "Starting measurement demonstration...");
    
    // Configure sensor for measurement
    status = VL53LX_SetDistanceMode(pdev, VL53LX_DISTANCEMODE_MEDIUM);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set distance mode: %d", status);
        return;
    }
    
    // Set timing budget (33ms default)
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(pdev, 33000);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set timing budget: %d", status);
        return;
    }
    
    // Start measurements
    status = VL53LX_StartMeasurement(pdev);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to start measurement: %d", status);
        return;
    }
    
    ESP_LOGI(TAG, "Measurements started. First measurement (Range1) will be discarded.");
    
    // Measurement loop demonstrating Range1 discard
    while (measurement_count < 10) {
        // Wait for measurement data ready
        uint8_t data_ready = 0;
        do {
            status = VL53LX_GetMeasurementDataReady(pdev, &data_ready);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "Failed to check data ready: %d", status);
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (!data_ready);
        
        // Get measurement data
        status = VL53LX_GetMultiRangingData(pdev, &data);
        if (status != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to get measurement data: %d", status);
            goto cleanup;
        }
        
        // CRITICAL: Always discard Range1 (no wrap-around validation)
        if (first_measurement) {
            ESP_LOGW(TAG, "Range1 DISCARDED (StreamCount: %d) - no wrap-around check performed", 
                     data.StreamCount);
            first_measurement = false;
        } else {
            // Process valid measurement data (Range2 onwards)
            measurement_count++;
            
            ESP_LOGI(TAG, "Measurement #%d (StreamCount: %d):", measurement_count, data.StreamCount);
            ESP_LOGI(TAG, "  Objects found: %d", data.NumberOfObjectsFound);
            
            if (data.NumberOfObjectsFound > 0) {
                for (int i = 0; i < data.NumberOfObjectsFound; i++) {
                    VL53LX_TargetRangeData_t *target = &data.RangeData[i];
                    
                    if (target->RangeStatus == 0) { // VL53LX_RANGESTATUS_RANGE_VALID
                        float signal_rate = (float)target->SignalRateRtnMegaCps / 65536.0f;
                        float sigma = (float)target->SigmaMilliMeter / 65536.0f;
                        
                        ESP_LOGI(TAG, "  Target %d: %d mm (Signal: %.2f MCps, Sigma: %.2f mm)", 
                                i, target->RangeMilliMeter, signal_rate, sigma);
                    } else {
                        ESP_LOGW(TAG, "  Target %d: Range status error %d", i, target->RangeStatus);
                    }
                }
            } else {
                ESP_LOGI(TAG, "  No targets detected");
            }
            
            // Check if crosstalk correction was applied
            if (data.HasXtalkValueChanged) {
                ESP_LOGI(TAG, "  Crosstalk correction applied this measurement");
            }
        }
        
        // Clear interrupt and start next measurement
        status = VL53LX_ClearInterruptAndStartMeasurement(pdev);
        if (status != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to clear interrupt: %d", status);
            goto cleanup;
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // 200ms between measurements
    }
    
cleanup:
    VL53LX_StopMeasurement(pdev);
    ESP_LOGI(TAG, "Measurement demonstration completed");
}

void app_main(void)
{
    VL53LX_Dev_t dev;
    VL53LX_DEV dev_handle = &dev;
    
    // Initialize NVS for calibration storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "VL53LX Production Calibration Example");
    ESP_LOGI(TAG, "====================================");
    
    // Set up device structure
    dev.i2c_slave_address = 0x52; // Default I2C address
    
    // Check if calibration data exists
    VL53LX_CalibrationData_t test_cal_data;
    bool calibration_exists = load_calibration_from_storage(&test_cal_data);
    
    if (!calibration_exists) {
        ESP_LOGW(TAG, "No calibration data found. Starting factory calibration procedure...");
        ESP_LOGW(TAG, "This requires manual setup for each calibration step.");
        
        // Initialize sensor for calibration
        if (VL53LX_CommsInitialise(dev_handle, 0, 400) != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to initialize communications for calibration");
            return;
        }
        
        if (VL53LX_WaitDeviceBooted(dev_handle) != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to wait for device boot");
            return;
        }
        
        if (VL53LX_DataInit(dev_handle) != VL53LX_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to initialize sensor data");
            return;
        }
        
        // Perform factory calibration
        if (!perform_factory_calibration(dev_handle)) {
            ESP_LOGE(TAG, "Factory calibration failed!");
            return;
        }
        
        ESP_LOGI(TAG, "Factory calibration completed. Restarting for production demo...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Production initialization with calibration loading
    production_sensor_init(dev_handle);
    
    // Demonstrate proper measurement handling
    demonstrate_measurement_with_range1_discard(dev_handle);
    
    ESP_LOGI(TAG, "Production calibration example completed successfully!");
}