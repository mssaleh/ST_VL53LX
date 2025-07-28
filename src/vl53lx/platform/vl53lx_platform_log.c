
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/* Platform logging implementation for ESP32 using ESP-IDF logging system
 * 
 * This implementation integrates the VL53LX logging with ESP-IDF's unified
 * logging framework, allowing users to control log levels via menuconfig
 * and providing consistent log formatting across the system.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "esp_log.h"
#include "esp_timer.h"
#include <vl53lx_platform_log.h>
#include <vl53lx_platform_user_config.h>


#ifdef VL53LX_LOG_ENABLE

/* ESP-IDF logging tag for VL53LX driver messages */
static const char *TAG = "VL53LX";

/* ESP-IDF doesn't require filename-based logging as logs go to console/UART */
uint32_t _trace_level     = VL53LX_TRACE_LEVEL_WARNING;
uint32_t _trace_modules   = VL53LX_TRACE_MODULE_NONE; 
uint32_t _trace_functions = VL53LX_TRACE_FUNCTION_ALL;

/* Map VL53LX trace levels to ESP log levels. */
static esp_log_level_t vl53lx_to_esp_log_level(uint32_t level)
{
    switch (level) {
        case VL53LX_TRACE_LEVEL_NONE:    return ESP_LOG_NONE;
        case VL53LX_TRACE_LEVEL_ERRORS:  return ESP_LOG_ERROR;
        case VL53LX_TRACE_LEVEL_WARNING: return ESP_LOG_WARN;
        case VL53LX_TRACE_LEVEL_INFO:    return ESP_LOG_INFO;
        case VL53LX_TRACE_LEVEL_DEBUG:   return ESP_LOG_DEBUG;
        case VL53LX_TRACE_LEVEL_ALL:     return ESP_LOG_VERBOSE;
        default:                         return ESP_LOG_INFO;
    }
}

int8_t VL53LX_trace_config(
    char *filename,
    uint32_t modules,
    uint32_t level,
    uint32_t functions)
{
    /* ESP-IDF logging doesn't use files - logs go to console/UART
     * The filename parameter is ignored for ESP32 platform.
     * Log level and module filtering can be controlled via menuconfig
     * or programmatically using esp_log_level_set().
     */
    (void)filename;
    
    _trace_modules   = modules;
    _trace_level     = level;
    _trace_functions = functions;
    
    /* Set ESP log level for the VL53LX component.
     * Only update if the new level is different to avoid unnecessary overhead.
     */
    esp_log_level_t new_esp_level = vl53lx_to_esp_log_level(level);
    esp_log_level_set(TAG, new_esp_level);
    
    /* Only log configuration changes when logging is actually enabled at info level
     * to avoid potential recursion or interference with system logging.
     */
    if (level >= VL53LX_TRACE_LEVEL_INFO) {
        ESP_LOGI(TAG, "VL53LX logging configured: level=%lu, modules=0x%lx, functions=0x%lx", 
                 level, modules, functions);
    }
    
    return 0;
}

void VL53LX_trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
{
    /* Check if this message should be printed based on current filter settings */
    if ( ((level <= _trace_level) && ((module & _trace_modules) > 0))
        || ((function & _trace_functions) > 0) )
    {
        va_list arg_list;
        char message[VL53LX_MAX_STRING_LENGTH];
        
        va_start(arg_list, format);
        vsnprintf(message, VL53LX_MAX_STRING_LENGTH, format, arg_list);
        va_end(arg_list);
        
        /* Ensure null termination in case of truncation */
        message[VL53LX_MAX_STRING_LENGTH-1] = '\0';
        
        /* Remove trailing newlines as ESP_LOG functions add them automatically */
        size_t len = strlen(message);
        while (len > 0 && (message[len-1] == '\n' || message[len-1] == '\r')) {
            message[--len] = '\0';
        }
        
        /* Skip logging if message was empty after cleanup */
        if (len == 0) {
            return;
        }
        
        /* Use appropriate ESP log level */
        esp_log_level_t esp_level = vl53lx_to_esp_log_level(level);
        ESP_LOG_LEVEL(esp_level, TAG, "%s", message);
    }
}

uint32_t VL53LX_get_trace_functions(void)
{
    return _trace_functions;
}

void VL53LX_set_trace_functions(uint32_t function)
{
    _trace_functions = function;
}

uint32_t VL53LX_clock(void)
{
    /* Use ESP timer for high-resolution timestamps */
    return (uint32_t)(esp_timer_get_time() / 1000);
}

#else

/* Stub implementations when logging is disabled */
int8_t VL53LX_trace_config(char *filename, uint32_t modules, uint32_t level, uint32_t functions)
{
    (void)filename; (void)modules; (void)level; (void)functions;
    return 0;
}

void VL53LX_trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
{
    (void)module; (void)level; (void)function; (void)format;
}

uint32_t VL53LX_get_trace_functions(void) { return 0; }
void VL53LX_set_trace_functions(uint32_t function) { (void)function; }
uint32_t VL53LX_clock(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

#endif

