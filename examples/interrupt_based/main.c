/*
 * VL53LX Interrupt-based measurement example for ESP32-C6
 * 
 * This example demonstrates how to use the VL53LX sensor in interrupt mode,
 * which is more efficient than polling and allows the system to perform other
 * tasks while waiting for measurements.
 * 
 * Hardware setup:
 * - Connect VL53LX SDA to GPIO 8 (or override via VL53LX_I2C_SDA)
 * - Connect VL53LX SCL to GPIO 9 (or override via VL53LX_I2C_SCL)
 * - Connect VL53LX GPIO1 (interrupt pin) to GPIO 10
 * - Connect VL53LX VDD to 3.3V
 * - Connect VL53LX GND to GND
 * - Optional: Connect VL53LX XSHUT to a GPIO for software reset
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "vl53lx_api.h"

static const char *TAG = "VL53LX_Interrupt";

/* GPIO configuration */
#define VL53LX_INT_PIN    GPIO_NUM_10
#define VL53LX_XSHUT_PIN  GPIO_NUM_11  // Optional reset pin

/* Sensor configuration */
#define SENSOR_I2C_ADDRESS  0x52

/* Global variables */
static VL53LX_Dev_t g_sensor_dev;
static VL53LX_DEV g_sensor_handle = &g_sensor_dev;
static SemaphoreHandle_t g_measurement_semaphore = NULL;
static QueueHandle_t g_measurement_queue = NULL;

/* Structure to pass measurement data between ISR and task */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t data_ready;
} measurement_event_t;

/* Function prototypes */
static void IRAM_ATTR vl53lx_int_handler(void *arg);
static void sensor_task(void *arg);
static void measurement_processor_task(void *arg);
static esp_err_t setup_interrupt_gpio(void);
static esp_err_t setup_reset_gpio(void);
static VL53LX_Error initialize_sensor(void);
static void reset_sensor(void);

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting VL53LX Interrupt-based Example");
    
    /* Initialize sensor device structure */
    g_sensor_dev.i2c_slave_address = SENSOR_I2C_ADDRESS;
    
    /* Create synchronization primitives */
    g_measurement_semaphore = xSemaphoreCreateBinary();
    if (g_measurement_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create measurement semaphore");
        return;
    }
    
    g_measurement_queue = xQueueCreate(10, sizeof(measurement_event_t));
    if (g_measurement_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create measurement queue");
        return;
    }
    
    /* Setup GPIO pins */
    ret = setup_reset_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup reset GPIO");
        return;
    }
    
    ret = setup_interrupt_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup interrupt GPIO");
        return;
    }
    
    /* Reset the sensor */
    reset_sensor();
    
    /* Create tasks */
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(measurement_processor_task, "meas_processor", 4096, NULL, 4, NULL, 1);
    
    ESP_LOGI(TAG, "Tasks created successfully");
}

static void IRAM_ATTR vl53lx_int_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    measurement_event_t event;
    
    /* Record timestamp */
    event.timestamp_ms = esp_timer_get_time() / 1000;
    event.data_ready = 1;
    
    /* Send event to queue from ISR */
    xQueueSendFromISR(g_measurement_queue, &event, &xHigherPriorityTaskWoken);
    
    /* Give semaphore to wake up the sensor task */
    xSemaphoreGiveFromISR(g_measurement_semaphore, &xHigherPriorityTaskWoken);
    
    /* Yield to higher priority task if necessary */
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void sensor_task(void *arg)
{
    VL53LX_Error status;
    uint32_t measurement_count = 0;
    
    ESP_LOGI(TAG, "Sensor task started");
    
    /* Initialize sensor communication and configuration */
    status = initialize_sensor();
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize sensor (error: %d)", status);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting interrupt-based measurements...");
    
    /* Start measurements */
    status = VL53LX_StartMeasurement(g_sensor_handle);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to start measurements (error: %d)", status);
        vTaskDelete(NULL);
        return;
    }
    
    /* Main sensor loop - wait for interrupts */
    while (1) {
        /* Wait for interrupt semaphore with timeout */
        if (xSemaphoreTake(g_measurement_semaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
            VL53LX_MultiRangingData_t data;
            uint8_t data_ready = 0;
            
            measurement_count++;
            
            /* Verify data is ready */
            status = VL53LX_GetMeasurementDataReady(g_sensor_handle, &data_ready);
            if (status == VL53LX_ERROR_NONE && data_ready) {
                /* Get measurement data */
                status = VL53LX_GetMultiRangingData(g_sensor_handle, &data);
                if (status == VL53LX_ERROR_NONE) {
                    ESP_LOGI(TAG, "Measurement #%lu: %u objects detected", 
                             measurement_count, data.NumberOfObjectsFound);
                    
                    if (data.NumberOfObjectsFound > 0) {
                        ESP_LOGI(TAG, "Primary object: %u mm (status: %u)", 
                                data.RangeData[0].RangeMilliMeter,
                                data.RangeData[0].RangeStatus);
                    }
                    
                    /* Clear interrupt and start next measurement */
                    VL53LX_ClearInterruptAndStartMeasurement(g_sensor_handle);
                } else {
                    ESP_LOGE(TAG, "Failed to get measurement data (error: %d)", status);
                }
            } else {
                ESP_LOGW(TAG, "Interrupt triggered but no data ready");
            }
        } else {
            ESP_LOGW(TAG, "Measurement timeout - no interrupt received");
            
            /* Check if sensor is still responsive */
            uint8_t device_ready = 0;
            status = VL53LX_GetMeasurementDataReady(g_sensor_handle, &device_ready);
            if (status != VL53LX_ERROR_NONE) {
                ESP_LOGE(TAG, "Sensor communication error (error: %d)", status);
                break;
            }
        }
    }
    
    /* Stop measurements before exiting */
    VL53LX_StopMeasurement(g_sensor_handle);
    ESP_LOGE(TAG, "Sensor task exiting due to error");
    vTaskDelete(NULL);
}

static void measurement_processor_task(void *arg)
{
    measurement_event_t event;
    uint32_t event_count = 0;
    uint32_t last_timestamp = 0;
    
    ESP_LOGI(TAG, "Measurement processor task started");
    
    while (1) {
        /* Wait for measurement events */
        if (xQueueReceive(g_measurement_queue, &event, portMAX_DELAY) == pdTRUE) {
            event_count++;
            
            uint32_t interval = (last_timestamp > 0) ? (event.timestamp_ms - last_timestamp) : 0;
            
            ESP_LOGI(TAG, "Event #%lu: timestamp=%lums, interval=%lums", 
                     event_count, event.timestamp_ms, interval);
            
            last_timestamp = event.timestamp_ms;
            
            /* Here you could add additional processing of measurement events
             * such as filtering, averaging, data logging, etc. */
        }
    }
}

static esp_err_t setup_interrupt_gpio(void)
{
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << VL53LX_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // VL53LX pulls low when data ready
    };
    
    esp_err_t ret = gpio_config(&int_conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    /* Install ISR service and add handler */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    
    ret = gpio_isr_handler_add(VL53LX_INT_PIN, vl53lx_int_handler, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Interrupt GPIO setup complete on pin %d", VL53LX_INT_PIN);
    return ESP_OK;
}

static esp_err_t setup_reset_gpio(void)
{
    gpio_config_t reset_conf = {
        .pin_bit_mask = (1ULL << VL53LX_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&reset_conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    /* Set pin high initially (sensor enabled) */
    gpio_set_level(VL53LX_XSHUT_PIN, 1);
    
    ESP_LOGI(TAG, "Reset GPIO setup complete on pin %d", VL53LX_XSHUT_PIN);
    return ESP_OK;
}

static VL53LX_Error initialize_sensor(void)
{
    VL53LX_Error status;
    
    /* Initialize I2C communication */
    status = VL53LX_CommsInitialise(g_sensor_handle, 0, 400);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize I2C (error: %d)", status);
        return status;
    }
    
    /* Wait for device to boot */
    status = VL53LX_WaitDeviceBooted(g_sensor_handle);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Device boot timeout (error: %d)", status);
        return status;
    }
    
    /* Initialize sensor data structures */
    status = VL53LX_DataInit(g_sensor_handle);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize data (error: %d)", status);
        return status;
    }
    
    /* Configure sensor for interrupt operation */
    status = VL53LX_SetDistanceMode(g_sensor_handle, VL53LX_DISTANCEMODE_SHORT);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set distance mode (error: %d)", status);
        return status;
    }
    
    /* Set measurement timing - faster for interrupt mode */
    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(g_sensor_handle, 50000); // 50ms
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set timing budget (error: %d)", status);
        return status;
    }
    
    /* Set inter-measurement period */
    status = VL53LX_SetInterMeasurementPeriodMilliSeconds(g_sensor_handle, 100);
    if (status != VL53LX_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set inter-measurement period (error: %d)", status);
        return status;
    }
    
    ESP_LOGI(TAG, "Sensor initialization completed successfully");
    return VL53LX_ERROR_NONE;
}

static void reset_sensor(void)
{
    ESP_LOGI(TAG, "Resetting sensor...");
    
    /* Pull XSHUT low to reset */
    gpio_set_level(VL53LX_XSHUT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Release reset */
    gpio_set_level(VL53LX_XSHUT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Sensor reset completed");
}
