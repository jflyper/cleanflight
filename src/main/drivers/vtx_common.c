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

#include "config/config_eeprom.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "vtx_common.h"
#include "vtx_debug.h"

#if defined(VTX_COMMON)

PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 0);

#if defined(VTX_RTC6705_SPI)
#define VTX_DEVICE VTX_DEVICE_RTC6705_SPI
#elif defined(VTX_RTC6705_SOFTSPI)
#define VTX_DEVICE VTX_DEVICE_RTC6705_SOFTSPI
#else
#define VTX_DEVICE VTX_DEVICE_OTHER
#endif

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
    .vtx_device = VTX_DEVICE,
    .vtx_mode = 0,
    .vtx_band = 1,
    .vtx_channel = 1,
    .vtx_mhz = 5740,
    .vtx_power = 1,
);

vtxConfig_t *pVtxConfig = NULL;

vtxDevice_t *pVtxDevice = NULL;

// Whatever registered last will win

void vtxCommonRegisterDevice(vtxDevice_t *pDevice)
{
    pVtxDevice = pDevice;

    // XXX Should we take care of devices that comes in "late"?
}

// XXX Periodical call (not yet)
void vtxCommonProcess(uint32_t currentTimeUs)
{
    if (!pVtxDevice)
        return;

    if (pVtxDevice->vTable->process) {
        pVtxDevice->vTable->process(currentTimeUs);
    }
}

vtxDevType_e vtxCommonGetDeviceType(void)
{
    if (!pVtxDevice || !pVtxDevice->vTable->getDeviceType)
        return VTXDEV_UNKNOWN;

    return pVtxDevice->vTable->getDeviceType();
}

void vtxCommonSetFselMode(uint8_t mode)
{
    if (!pVtxDevice)
        return;

    if (pVtxDevice->vTable->setFselMode)
        pVtxDevice->vTable->setFselMode(mode);
}

// band and chan are 1 origin
void vtxCommonSetBandChan(uint8_t band, uint8_t chan)
{
    if (!pVtxDevice)
        return;

    if (pVtxConfig->vtx_mode != 0)
        return;

    if ((band > pVtxDevice->devParam.numBand)|| (chan > pVtxDevice->devParam.numChan))
        return;
    
    if (pVtxDevice->vTable->setBandChan)
        pVtxDevice->vTable->setBandChan(band, chan);
}

void vtxCommonSetFreq(uint16_t freq)
{
    dprintf(("vtxCommonSetFreq: top\r\n"));
    if (!pVtxDevice)
        return;

    dprintf(("vtxCommonSetFreq: checking vtx_mode\r\n"));
    if (pVtxConfig->vtx_mode != 1)
        return;

    dprintf(("vtxCommonSetFreq: checking freq range\r\n"));

    // XXX Frequency range must be a part of devParam
    if ((freq < 5600)|| (freq > 5950))
        return;

    dprintf(("vtxCommonSetFreq: setFreq\r\n"));

    if (pVtxDevice->vTable->setFreq)
        pVtxDevice->vTable->setFreq(freq);
}

// index is zero origin, zero = power off completely
void vtxCommonSetPowerByIndex(uint8_t index)
{
    if (!pVtxDevice)
        return;

    if (index > pVtxDevice->devParam.numPower)
        return;
    
    if (pVtxDevice->vTable->setPowerByIndex)
        pVtxDevice->vTable->setPowerByIndex(index);
}

// on = 1, off = 0
void vtxCommonSetPitmode(uint8_t onoff)
{
    if (!pVtxDevice)
        return;

    if (pVtxDevice->vTable->setPitmode)
        pVtxDevice->vTable->setPitmode(onoff);
}

// band/channel = 0, direct frequency = 1
bool vtxCommonGetFselMode(uint8_t *pMode)
{
    if (!pVtxDevice)
        return false;

    if (pVtxDevice->vTable->getFselMode)
        return pVtxDevice->vTable->getFselMode(pMode);
    else
        return false;
}

bool vtxCommonGetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    if (!pVtxDevice)
        return false;

    if (pVtxDevice->vTable->getBandChan)
        return pVtxDevice->vTable->getBandChan(pBand, pChan);
    else
        return false;
}

bool vtxCommonGetFreq(uint16_t *pFreq)
{
    if (!pVtxDevice)
        return false;

    if (pVtxDevice->vTable->getFreq) {
        return pVtxDevice->vTable->getFreq(pFreq);
    }
    else
        return false;
}

bool vtxCommonGetPowerIndex(uint8_t *pIndex)
{
    if (!pVtxDevice)
        return false;

    if (pVtxDevice->vTable->getPowerIndex)
        return pVtxDevice->vTable->getPowerIndex(pIndex);
    else
        return false;
}

bool vtxCommonGetPitmode(uint8_t *pOnoff)
{
    if (!pVtxDevice)
        return false;

    if (pVtxDevice->vTable->getPitmode)
        return pVtxDevice->vTable->getPitmode(pOnoff);
    else
        return false;
}

vtxDeviceParam_t *vtxCommonGetDeviceParam(void)
{
    if (!pVtxDevice)
        return NULL;

    return &pVtxDevice->devParam;
}

void vtxCommonInit(vtxConfig_t *vtxConfigMutableToUse)
{
    pVtxConfig = vtxConfigMutableToUse;

    if (!pVtxDevice) {
        dprintf(("vtxCommonInit: no vtxDevice\r\n"));
        return;
    }

    dprintf(("vtxCommonInit: vtx_mode %d vtx_band %d vtx_chan %d vtx_mhz %d vtx_power %d\r\n", pVtxConfig->vtx_mode, pVtxConfig->vtx_band, pVtxConfig->vtx_channel, pVtxConfig->vtx_mhz, pVtxConfig->vtx_power));
}

#endif
