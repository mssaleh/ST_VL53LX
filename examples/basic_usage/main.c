/*
 * Example app demonstrating how to use the VL53LX library with
 * ESP‑IDF in a PlatformIO environment.  This example initialises the
 * sensor, starts a measurement and periodically prints the distance in
 * millimetres.  Connect the sensor’s SDA and SCL pins to the GPIOs
 * defined by VL53LX_I2C_SDA and VL53LX_I2C_SCL (or override them
 * in platformio.ini).  Don’t forget to power the sensor and connect
 * ground.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l3cx_api.h"

static const char *TAG = "VL53LX_Basic";

static void vl53lx_task(void *arg)
{
    VL53LX_Dev_t dev;
    VL53LX_DEV dev_handle = &dev;

    /* Use the default I2C address.  The VL53LX has a fixed I2C address
     * that cannot be changed. ST API expects 8-bit format (0x52), which
     * the platform layer automatically converts to 7-bit for ESP-IDF (0x29).
     */
    dev.i2c_slave_address = 0x52;

    if (VL53LX_CommsInitialise(dev_handle, 0, 400) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialise I2C communication");
        vTaskDelete(NULL);
    }

    if (VL53LX_WaitDeviceBooted(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_DataInit(dev_handle) != VL53LX_ERROR_NONE ||
        VL53LX_SetDistanceMode(dev_handle, VL53LX_DISTANCEMODE_SHORT) != VL53LX_ERROR_NONE ||
        VL53LX_StartMeasurement(dev_handle) != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Sensor initialisation failed");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "VL53LX sensor initialized successfully");

    while (1) {
        VL53LX_MultiRangingData_t data;
        if (VL53LX_GetMultiRangingData(dev_handle, &data) == VL53LX_ERROR_NONE &&
            data.NumberOfObjectsFound > 0) {
            ESP_LOGI(TAG, "Distance: %u mm (status: %u)", 
                     data.RangeData[0].RangeMilliMeter, 
                     data.RangeData[0].RangeStatus);
        } else {
            ESP_LOGD(TAG, "No objects detected");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "VL53LX Basic Example Starting");
    xTaskCreatePinnedToCore(vl53lx_task, "vl53lx", 4096, NULL, 5, NULL, 0);
}