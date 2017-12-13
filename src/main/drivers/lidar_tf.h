/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"
#include "drivers/altimeter.h"

typedef struct lidarTFConfig_s {
    uint8_t device;
} lidarTFConfig_t;

#define LIDAR_TF_TYPE_TFMINI 0
#define LIDAR_TF_TYPE_TF02   1

PG_DECLARE(lidarTFConfig_t, lidarTFConfig);

altimeterDevice_t *lidarTFInit(void);
