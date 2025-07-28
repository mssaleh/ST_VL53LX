/*
 * VL53LX Time-of-Flight Ranging Sensor Library for ESP-IDF
 * 
 * This is the main header file for the VL53LX ToF sensor library.
 * Include this file in your ESP-IDF project to access all VL53LX functionality.
 * 
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#ifndef _VL53LX_API_WRAPPER_H_
#define _VL53LX_API_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Include the core VL53LX API */
#include "vl53lx/core/vl53lx_api.h"

/* Include platform definitions */
#include "vl53lx/platform/vl53lx_platform.h"
#include "vl53lx/platform/vl53lx_types.h"

#ifdef __cplusplus
}
#endif

#endif /* _VL53LX_API_WRAPPER_H_ */
