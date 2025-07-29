/*
 * VL53L3CX Time-of-Flight Ranging Sensor Library for ESP-IDF
 * 
 * This is the main header file for the VL53L3CX ToF sensor library.
 * Include this file in your ESP-IDF project to access all VL53LX functionality.
 * 
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#ifndef _VL53L3CX_API_H_
#define _VL53L3CX_API_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Include the core VL53LX API */
#include "vl53lx/core/vl53lx_api.h"

/* Include platform definitions */
#include "vl53lx/platform/vl53lx_platform.h"

#ifdef __cplusplus
}
#endif

#endif /* _VL53L3CX_API_H_ */