// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Platform port layer for the ST VL53LX ranging sensor.
 *
 * This file provides an implementation of the low‑level communication
 * primitives required by the ST VL53LX driver.  The original ST
 * sources ship with empty stubs that must be filled in by the user.  The
 * implementation below targets the Espressif ESP32 family and relies
 * on the ESP‑IDF new I2C master driver.  It should work on other
 * ESP32 variants as well but may require adjustment of the I2C pin
 * assignments.
 *
 * The goal of this port is to remain as close as possible to the
 * recommended usage demonstrated in the official ESP‑IDF I2C API
 * documentation.  See the “I2C master write” and “I2C master read”
 * examples in the ESP‑IDF guide for details【462041596841165†L425-L447】【462041596841165†L489-L523】.
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"

#include "vl53lx_platform.h"

/*
 * The VL53LX driver stores the I2C address and bus configuration in the
 * VL53LX_Dev_t structure.  However, it does not provide storage for
 * platform‑specific handles such as the I2C bus or device handle.  We
 * therefore use static variables to keep track of the bus and device
 * handles created during initialisation.  This implementation assumes
 * that only one VL53LX device is attached to a single I2C bus.  If
 * multiple devices are used, it would be necessary to extend the
 * VL53LX_Dev_t structure to hold per‑device handles, or wrap the global
 * handles in a higher‑level context.
 */
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_i2c_dev_handle = NULL;

/* Default I2C configuration
 *
 * ESP32 boards does not reserve fixed pins for I2C.
 * Instead, any free GPIOs may be used.  Users can override these
 * values by defining VL53LX_I2C_SDA and VL53LX_I2C_SCL in the
 * PlatformIO build flags (see README.md for details).
 */
#ifndef VL53LX_I2C_PORT
#define VL53LX_I2C_PORT 0
#endif

#ifndef VL53LX_I2C_SDA
#define VL53LX_I2C_SDA 8
#endif

#ifndef VL53LX_I2C_SCL
#define VL53LX_I2C_SCL 9
#endif

/*
 * Convert ESP‑IDF error codes into VL53LX error codes.
 *
 * The ST driver uses negative error values with type VL53LX_Error
 * defined in vl53lx_error_codes.h.  We map ESP_OK to VL53LX_ERROR_NONE and
 * all other errors to VL53LX_ERROR_CONTROL_INTERFACE.
 */
static inline VL53LX_Error esp_to_vl53lx_error(esp_err_t err)
{
    return (err == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t       comms_type,
    uint16_t      comms_speed_khz)
{
    /* Only I2C is supported by this implementation. */
    (void)comms_type;

    /* If a bus has already been initialised, simply return success. */
    if (s_i2c_dev_handle != NULL) {
        return VL53LX_ERROR_NONE;
    }

    /* Configure the I2C bus.  The clock source is left at the default
     * setting.  We enable internal pull‑ups on the pins because
     * external pull‑ups may not always be fitted on development
     * boards.  According to the ESP‑IDF docs, the bus must be
     * installed before devices can be attached.
     */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = VL53LX_I2C_PORT,
        .scl_io_num = VL53LX_I2C_SCL,
        .sda_io_num = VL53LX_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    /* Configure the device on the bus.  The device address and clock speed
     * come from the VL53LX_Dev_t structure.  The ST driver stores the
     * I2C speed in kHz, but ESP‑IDF expects it in Hz.
     * 
     * CRITICAL: ST API uses 8-bit I2C addressing (includes R/W bit), but
     * ESP-IDF I2C driver expects 7-bit addresses. Convert by right-shifting.
     * Example: ST API address 0x52 -> ESP-IDF address 0x29 (0x52 >> 1)
     */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = pdev->i2c_slave_address >> 1,  // Convert 8-bit to 7-bit
        .scl_speed_hz = (uint32_t)comms_speed_khz * 1000U,
    };

    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_cfg, &s_i2c_dev_handle);
    return esp_to_vl53lx_error(ret);
}

VL53LX_Error VL53LX_CommsClose(
    VL53LX_Dev_t *pdev)
{
    (void)pdev;
    
    /* Clean up device handle first, then bus handle.
     * Ignore errors during cleanup to ensure both resources are freed.
     */
    if (s_i2c_dev_handle) {
        i2c_master_bus_rm_device(s_i2c_dev_handle);
        s_i2c_dev_handle = NULL;
    }
    if (s_i2c_bus_handle) {
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    (void)pdev;
    if (!s_i2c_dev_handle || !pdata || count == 0) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    /* Compose a buffer containing the 16‑bit register index followed by
     * the payload.  The VL53LX register map uses big‑endian addressing
     * (MSB first).  See the ST API for details.
     * 
     * Use dynamic allocation to avoid VLA and potential stack overflow
     * for large transfers. Add bounds checking for safety.
     */
    if (count > 1024) {  // Reasonable limit for sensor operations
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    
    uint8_t *buf = malloc(2 + count);
    if (!buf) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&buf[2], pdata, count);

    /* Use a reasonable timeout instead of infinite to prevent hangs.
     * 5000ms should be sufficient for sensor operations while allowing
     * the system to remain responsive.
     */
    esp_err_t ret = i2c_master_transmit(s_i2c_dev_handle, buf, 2 + count, 5000);
    
    free(buf);
    return esp_to_vl53lx_error(ret);
}

VL53LX_Error VL53LX_ReadMulti(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    (void)pdev;
    if (!s_i2c_dev_handle || !pdata || count == 0) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    /* Create a two‑byte buffer containing the register index. */
    uint8_t idx[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
    /* Perform a combined write/read transaction.  According to the ESP‑IDF
     * documentation this function will send the index bytes and then read
     * the requested number of bytes from the device without releasing
     * the bus in between. Use reasonable timeout to prevent system hangs.
     */
    esp_err_t ret = i2c_master_transmit_receive(s_i2c_dev_handle,
                                                idx, sizeof(idx),
                                                pdata, count,
                                                5000);
    return esp_to_vl53lx_error(ret);
}

/* Single byte/word/dword helpers use the multi‑byte routines above. */

VL53LX_Error VL53LX_WrByte(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t       data)
{
    return VL53LX_WriteMulti(pdev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint16_t      data)
{
    uint8_t buffer[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53LX_WriteMulti(pdev, index, buffer, 2);
}

VL53LX_Error VL53LX_WrDWord(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint32_t      data)
{
    uint8_t buffer[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)(data & 0xFF)
    };
    return VL53LX_WriteMulti(pdev, index, buffer, 4);
}

VL53LX_Error VL53LX_RdByte(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata)
{
    return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint16_t     *pdata)
{
    uint8_t buffer[2];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 2);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    }
    return status;
}

VL53LX_Error VL53LX_RdDWord(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint32_t     *pdata)
{
    uint8_t buffer[4];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 4);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8)  |
                 (uint32_t)buffer[3];
    }
    return status;
}

/* Timing functions
 *
 * The ST driver expects WaitUs and WaitMs to block for at least the
 * requested duration and return zero on success.  We use esp_rom_delay_us()
 * for microsecond delays and vTaskDelay() for millisecond delays.  Note
 * that vTaskDelay() has a resolution of the FreeRTOS tick period, so
 * the actual delay may be slightly longer than requested.  According
 * to the ESP‑IDF documentation, this is suitable for low frequency
 * delays.
 */
VL53LX_Error VL53LX_WaitUs(
    VL53LX_Dev_t *pdev,
    int32_t       wait_us)
{
    (void)pdev;
    if (wait_us <= 0) {
        return VL53LX_ERROR_NONE;
    }
    esp_rom_delay_us((uint32_t)wait_us);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(
    VL53LX_Dev_t *pdev,
    int32_t       wait_ms)
{
    (void)pdev;
    if (wait_ms <= 0) {
        return VL53LX_ERROR_NONE;
    }
    /* Convert milliseconds to FreeRTOS ticks and delay.  Add one tick
     * to ensure at least the requested time has passed.
     */
    TickType_t ticks = pdMS_TO_TICKS(wait_ms);
    vTaskDelay(ticks == 0 ? 1 : ticks);
    return VL53LX_ERROR_NONE;
}

/* Timer utility functions
 *
 * Provide current tick count in milliseconds.  Use esp_timer_get_time()
 * which returns microseconds since boot.  Convert to milliseconds.
 */
#include "esp_timer.h"

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    if (!ptimer_freq_hz) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    *ptimer_freq_hz = 1000; /* Millisecond tick frequency */
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    if (!ptimer_count) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    int64_t us = esp_timer_get_time();
    *ptimer_count = (int32_t)(us / 1000);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(
    VL53LX_Dev_t *pdev,
    uint32_t     *ptick_count_ms)
{
    (void)pdev;
    if (!ptick_count_ms) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    int64_t us = esp_timer_get_time();
    *ptick_count_ms = (uint32_t)(us / 1000);
    return VL53LX_ERROR_NONE;
}

/* ESP32 GPIO implementation using ESP-IDF GPIO driver
 *
 * These functions provide GPIO control for the VL53LX sensor pins
 * including reset (XSHUT), power control, and interrupt handling.
 * Pin assignments can be configured via build flags in platformio.ini:
 * 
 * -DVL53LX_XSHUT_PIN=11    (reset pin)
 * -DVL53LX_INT_PIN=10      (interrupt pin) 
 * -DVL53LX_PWR_PIN=12      (power enable pin, optional)
 */
#include "driver/gpio.h"

/* Default GPIO pin assignments for ESP32-C6 DevKitM-1 */
#ifndef VL53LX_XSHUT_PIN
#define VL53LX_XSHUT_PIN GPIO_NUM_11
#endif

#ifndef VL53LX_INT_PIN  
#define VL53LX_INT_PIN GPIO_NUM_10
#endif

#ifndef VL53LX_PWR_PIN
#define VL53LX_PWR_PIN GPIO_NUM_12
#endif

/* Store interrupt callback and GPIO initialization state for each pin */
static void (*s_interrupt_callback)(void) = NULL;
static bool s_xshut_initialized = false;
static bool s_power_initialized = false;

/* Initialize GPIO pin with specified mode */
static esp_err_t init_gpio_pin(gpio_num_t pin, gpio_mode_t mode, 
                               gpio_pull_mode_t pull, gpio_int_type_t intr_type)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = mode,
        .pull_up_en = (pull == GPIO_PULLUP_ONLY || pull == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (pull == GPIO_PULLDOWN_ONLY || pull == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = intr_type
    };
    return gpio_config(&io_conf);
}

/* Internal interrupt handler that calls the registered callback */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    if (s_interrupt_callback) {
        s_interrupt_callback();
    }
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    esp_err_t ret;
    gpio_num_t gpio_pin = (gpio_num_t)pin;
    
    /* Mode mapping: 0=Input, 1=Output, 2=Input with pullup */
    switch (mode) {
        case 0: /* Input */
            ret = init_gpio_pin(gpio_pin, GPIO_MODE_INPUT, GPIO_FLOATING, GPIO_INTR_DISABLE);
            break;
        case 1: /* Output */ 
            ret = init_gpio_pin(gpio_pin, GPIO_MODE_OUTPUT, GPIO_FLOATING, GPIO_INTR_DISABLE);
            break;
        case 2: /* Input with pullup */
            ret = init_gpio_pin(gpio_pin, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY, GPIO_INTR_DISABLE);
            break;
        default:
            return VL53LX_ERROR_INVALID_PARAMS;
    }
    
    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    esp_err_t ret = gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    if (!pvalue) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    
    int level = gpio_get_level((gpio_num_t)pin);
    *pvalue = (level > 0) ? 1 : 0;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    if (!s_xshut_initialized) {
        esp_err_t ret = init_gpio_pin(VL53LX_XSHUT_PIN, GPIO_MODE_OUTPUT, GPIO_FLOATING, GPIO_INTR_DISABLE);
        if (ret != ESP_OK) {
            return VL53LX_ERROR_CONTROL_INTERFACE;
        }
        s_xshut_initialized = true;
    }
    
    esp_err_t ret = gpio_set_level(VL53LX_XSHUT_PIN, value ? 1 : 0);
    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    /* VL53LX uses I2C by default. This function is typically used
     * to switch between I2C and SPI modes, but since this implementation
     * only supports I2C, we simply return success.
     */
    (void)value;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    /* Enable/disable power to the sensor via power control pin.
     * This is optional and depends on your hardware design.
     */
    if (!s_power_initialized) {
        esp_err_t ret = init_gpio_pin(VL53LX_PWR_PIN, GPIO_MODE_OUTPUT, GPIO_FLOATING, GPIO_INTR_DISABLE);
        if (ret != ESP_OK) {
            return VL53LX_ERROR_CONTROL_INTERFACE;
        }
        /* Set initial state to powered off */
        gpio_set_level(VL53LX_PWR_PIN, 0);
        s_power_initialized = true;
    }
    
    esp_err_t ret = gpio_set_level(VL53LX_PWR_PIN, value ? 1 : 0);
    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    if (!function) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }
    
    /* Configure interrupt pin as input with pullup */
    esp_err_t ret = init_gpio_pin(VL53LX_INT_PIN, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY, 
                                  (edge_type == 0) ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE);
    if (ret != ESP_OK) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    /* Install ISR service if not already done. ESP_ERR_INVALID_STATE
     * indicates the service is already installed, which is acceptable.
     */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    /* Store callback and add interrupt handler */
    s_interrupt_callback = function;
    ret = gpio_isr_handler_add(VL53LX_INT_PIN, gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }
    
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    /* Clear callback first to prevent spurious interrupts during cleanup */
    s_interrupt_callback = NULL;
    esp_err_t ret = gpio_isr_handler_remove(VL53LX_INT_PIN);
    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    /*
     * Poll a register until a masked value matches the expected value or
     * a timeout occurs.  This is used by the ST driver for various
     * readiness checks.  We implement this using VL53LX_GetTickCount()
     * and repeated reads.  A small delay is inserted between
     * iterations to avoid saturating the bus.
     */
    uint32_t start_ms = 0;
    VL53LX_Error status = VL53LX_GetTickCount(pdev, &start_ms);
    if (status != VL53LX_ERROR_NONE) {
        return status;
    }
    uint8_t byte_value = 0;
    uint32_t current_ms = start_ms;
    do {
        status = VL53LX_RdByte(pdev, index, &byte_value);
        if (status != VL53LX_ERROR_NONE) {
            return status;
        }
        if ((byte_value & mask) == value) {
            return VL53LX_ERROR_NONE;
        }
        VL53LX_WaitMs(pdev, poll_delay_ms);
        status = VL53LX_GetTickCount(pdev, &current_ms);
        if (status != VL53LX_ERROR_NONE) {
            return status;
        }
    } while ((current_ms - start_ms) < timeout_ms);
    return VL53LX_ERROR_TIME_OUT;
}