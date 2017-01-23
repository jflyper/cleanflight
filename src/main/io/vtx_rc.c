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

/*
 * Originally created by S. Blakemore@IMPULSERC
 */

// Provides stick/switch UI for VTX configuration

#include "platform.h"

#ifdef USE_VTX_RC

// Own interfaces
#include "io/vtx_rc.h"

//External dependencies
#include "config/config_master.h"
#include "config/config_eeprom.h"
#include "drivers/vtx_common.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"

static uint8_t locked = 0;

static uint8_t numBand;
static uint8_t numChannel;

static void setChannelSaveAndNotify(uint8_t *bandOrChannel, uint8_t step, int32_t min, int32_t max)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (masterConfig.vtx_mode == 0 && !locked) {
        uint8_t temp = (*bandOrChannel) + step;
        temp = constrain(temp, min, max);
        *bandOrChannel = temp;

        vtxCommonSetBandChan(masterConfig.vtx_band, masterConfig.vtx_channel);
        writeEEPROM();
        readEEPROM();
        beeperConfirmationBeeps(temp);
    }
}

void vtxRcIncrementBand(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), 1, 1, numBand);
}

void vtxRcDecrementBand(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), -1, 1, numBand);
}

void vtxRcIncrementChannel(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), 1, 1, numChannel);
}

void vtxRcDecrementChannel(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), -1, 1, numChannel);
}

void vtxRcUpdateActivatedChannel(void)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (masterConfig.vtx_mode == 2 && !locked) {
        static uint8_t lastIndex = -1;
        uint8_t index;

        for (index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            vtxRcChannelActivationCondition_t *vtxRcChannelActivationCondition = &masterConfig.vtxRcChannelActivationConditions[index];

            if (isRangeActive(vtxRcChannelActivationCondition->auxChannelIndex, &vtxRcChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;
                vtxCommonSetBandChan(vtxRcChannelActivationCondition->band, vtxRcChannelActivationCondition->channel);
                break;
            }
        }
    }
}

void vtxRcInit(void)
{
    // What if this call fail?
    vtxCommonGetParam(&numBand, &numChannel, NULL, NULL, NULL, NULL);
}
#endif
