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

#include "fc/rc_controls.h"

#if 0
#define VTX_BAND_MIN                            1
#define VTX_BAND_MAX                            5
#define VTX_CHANNEL_MIN                         1
#define VTX_CHANNEL_MAX                         8
#endif

#define MAX_CHANNEL_ACTIVATION_CONDITION_COUNT  10

typedef struct vtxRcChannelActivationCondition_s {
    uint8_t auxChannelIndex;
    uint8_t band;
    uint8_t channel;
    channelRange_t range;
} vtxRcChannelActivationCondition_t;

void vtxRcIncrementBand(void);
void vtxRcDecrementBand(void);
void vtxRcIncrementChannel(void);
void vtxRcDecrementChannel(void);
void vtxRcUpdateActivatedChannel(void);

