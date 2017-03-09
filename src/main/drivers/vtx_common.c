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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"
#include "build/debug.h"

#if defined(VTX_COMMON)

#include "fc/config.h"
#include "vtx_common.h"
#include "vtx_debug.h"

vtxDevice_t *vtxDevice = NULL;
vtxConfig_t *pVtxConfig = NULL;


// Whatever registered last will win

void vtxCommonRegisterDevice(vtxDevice_t *pDevice)
{
    vtxDevice = pDevice;

    // XXX Should we take care of devices that comes in "late"?
}

// XXX Periodical call (not yet)
void vtxCommonProcess(uint32_t currentTimeUs)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->process) {
        vtxDevice->vTable->process(currentTimeUs);
    }
}

vtxDevType_e vtxCommonGetDeviceType(void)
{
    if (!vtxDevice || !vtxDevice->vTable->getDeviceType)
        return VTXDEV_UNKNOWN;

    return vtxDevice->vTable->getDeviceType();
}

void vtxCommonSetFselMode(uint8_t mode)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setFselMode)
        vtxDevice->vTable->setFselMode(mode);
}

// band and chan are 1 origin
void vtxCommonSetBandChan(uint8_t band, uint8_t chan)
{
    if (!vtxDevice)
        return;

    if (pVtxConfig->vtx_mode != 0)
        return;

    if ((band > vtxDevice->numBand)|| (chan > vtxDevice->numChan))
        return;
    
    if (vtxDevice->vTable->setBandChan)
        vtxDevice->vTable->setBandChan(band, chan);

    pVtxConfig->vtx_band = band;
    pVtxConfig->vtx_channel = chan;

    dprintf(("vtxCommonSetBandChan: saving config\r\n"));
    saveConfigAndNotify();

    //writeEEPROM();
    //readEEPROM();
}

// index is zero origin, zero = power off completely
void vtxCommonSetPowerByIndex(uint8_t index)
{
    if (!vtxDevice)
        return;

    if (index > vtxDevice->numPower)
        return;
    
    if (vtxDevice->vTable->setPowerByIndex)
        vtxDevice->vTable->setPowerByIndex(index);

    pVtxConfig->vtx_power = index;

    writeEEPROM();
    //readEEPROM();
}

void vtxCommonSetFreq(uint16_t freq)
{
    if (!vtxDevice)
        return;

    if (pVtxConfig->vtx_mode != 1)
        return;

    if ((freq < 5600)|| (freq > 5950))
        return;

    if (vtxDevice->vTable->setFreq)
        vtxDevice->vTable->setFreq(freq);

    pVtxConfig->vtx_mhz = freq;

    writeEEPROM();
    //readEEPROM();
}

// on = 1, off = 0
void vtxCommonSetPitmode(uint8_t onoff)
{
    if (!vtxDevice)
        return;

    if (vtxDevice->vTable->setPitmode)
        vtxDevice->vTable->setPitmode(onoff);
}

// band/channel = 0, direct frequency = 1
bool vtxCommonGetFselMode(uint8_t *pMode)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getBandChan)
        return vtxDevice->vTable->getFselMode(pMode);
    else
        return false;
}

bool vtxCommonGetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getBandChan)
        return vtxDevice->vTable->getBandChan(pBand, pChan);
    else
        return false;
}

bool vtxCommonGetPowerIndex(uint8_t *pIndex)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getPowerIndex)
        return vtxDevice->vTable->getPowerIndex(pIndex);
    else
        return false;
}

bool vtxCommonGetPitmode(uint8_t *pOnoff)
{
    if (!vtxDevice)
        return false;

    if (vtxDevice->vTable->getPitmode)
        return vtxDevice->vTable->getPitmode(pOnoff);
    else
        return false;
}

bool vtxCommonGetParam(uint8_t *pNumBand, uint8_t *pNumChan, uint8_t *pNumPower, char ***pBandNames, char ***pChanNames, char ***pPowerNames)
{
    if (!vtxDevice)
        return false;

    if (pNumBand)
        *pNumBand = vtxDevice->numBand;
    if (pNumChan)
        *pNumChan = vtxDevice->numChan;
    if (pNumPower)
        *pNumPower = vtxDevice->numPower;
    if (pBandNames)
        *pBandNames = vtxDevice->bandNames;
    if (pChanNames)
        *pChanNames = vtxDevice->chanNames;
    if (pPowerNames)
        *pPowerNames = vtxDevice->powerNames;

    return true;
}

void vtxCommonInit(vtxConfig_t *pVtxConfigToUse)
{
    pVtxConfig = pVtxConfigToUse;

    if (!vtxDevice) {
        dprintf(("vtxCommonInit: no vtxDevice\r\n"));
        return;
    }

    dprintf(("vtxCommonInit: vtx_mode %d vtx_band %d vtx_chan %d vtx_mhz %d\r\n", pVtxConfig->vtx_mode, pVtxConfig->vtx_band, pVtxConfig->vtx_channel, pVtxConfig->vtx_mhz));

    // Initialize vtxDevice according to the vtxConfig.

    switch (pVtxConfig->vtx_mode) {
    case 0: // Band/channel mode
        vtxCommonSetFselMode(0);
        vtxCommonSetBandChan(pVtxConfig->vtx_band, pVtxConfig->vtx_channel);
        break;

    case 1: // Direct frequency mode
        vtxCommonSetFselMode(1);
        break;

    case 2: // AUX switch mode, turn on band/channel mode
        vtxCommonSetFselMode(0);
        break;
    }
}
#endif
