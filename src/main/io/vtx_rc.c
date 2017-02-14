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

#include "build/debug.h"

// Own interfaces
#include "io/vtx_rc.h"

//External dependencies
#include "config/config_master.h"
#include "config/config_eeprom.h"
#include "drivers/vtx_common.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"

static vtxConfig_t *pVtxConfig = NULL;

static uint8_t locked = 0;

static uint8_t numBand;
static uint8_t numChannel;

static uint8_t curBand;
static uint8_t curChan;

static uint8_t reqBand;
static uint8_t reqChan;

static void vtxRcChangeBand(int delta)
{
    if (!pVtxConfig && pVtxConfig->vtx_mode != 0)
        return;

    if (!vtxCommonGetBandChan(&curBand, &curChan))
        return;

    reqBand = constrain(curBand + delta, 1, numBand);

    vtxCommonSetBandChan(reqBand, curChan);

    if (!vtxCommonGetBandChan(&curBand, &curChan))
        return;

    if (curBand != reqBand)
        return;

    beeperConfirmationBeeps(curBand);
}

static void vtxRcChangeChan(int delta)
{
    if (!pVtxConfig && pVtxConfig->vtx_mode != 0)
        return;

    if (!vtxCommonGetBandChan(&curBand, &curChan))
        return;

    reqChan = constrain(curChan + delta, 1, numChannel);

    vtxCommonSetBandChan(curBand, reqChan);

    if (!vtxCommonGetBandChan(&curBand, &curChan))
        return;

    if (curChan != reqChan)
        return;

    beeperConfirmationBeeps(curChan);
}

void vtxRcIncrementBand(void)
{
    vtxRcChangeBand(1);
}

void vtxRcDecrementBand(void)
{
    vtxRcChangeBand(-1);
}

void vtxRcIncrementChannel(void)
{
    vtxRcChangeChan(1);
}

void vtxRcDecrementChannel(void)
{
    vtxRcChangeChan(-1);
}

void vtxRcUpdateActivatedChannel(void)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (pVtxConfig && pVtxConfig->vtx_mode == 2 && !locked)
    {
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

void vtxRcInit(vtxConfig_t *pVtxConfigToUse)
{
    pVtxConfig = pVtxConfigToUse;

    // XXX What if this call fail?
    vtxCommonGetParam(&numBand, &numChannel, NULL, NULL, NULL, NULL);
}
#endif
