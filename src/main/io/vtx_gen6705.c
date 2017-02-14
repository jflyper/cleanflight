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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef VTX_RTC6705

#include "build/debug.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "drivers/vtx_common.h"
#include "io/vtx_gen6705.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

gen6705Device_t *pDevice = NULL;

// Downward API

void gen6705RegisterDevice(gen6705Device_t *pGen6705Device)
{
    pDevice = pGen6705Device;
}

static void gen6705WriteRegister(uint8_t addr, uint32_t data)
{
    if (pDevice && pDevice->writeRegister)
        (*pDevice->writeRegister)(addr, data);
}

// Upward API

static vtxVTable_t gen6705VTable;

static char *gen6705PowerNames[] = {
    "MIN",
    "LOW",
    "MID",
    "HIGH"
};

static vtxDevice_t gen6705Device = {
    .vTable = &gen6705VTable,

    .numBand = 5,
    .numChan = 8,
    .numPower = 4,

    .bandNames = (char **)vtx58BandNames,
    .chanNames = (char **)vtx58ChannelNames,
    .powerNames = (char **)gen6705PowerNames,
};

static bool gen6705IsReady(void)
{
    return true;
}

static vtxDevType_e gen6705GetDeviceType(void)
{
    return VTXDEV_GEN6705;
}

void gen6705SetFreq(uint16_t channel_freq)
{
    uint32_t freq = (uint32_t)channel_freq * 1000;
    uint32_t N, A;

    freq /= 40;
    N = freq / 64;
    A = freq % 64;
    gen6705WriteRegister(0, 400);
    gen6705WriteRegister(1, (N << 7) | A);
}

void gen6705SetBandChan(uint8_t band, uint8_t chan)
{
    debug[0]++;
    debug[1] = band;
    debug[2] = chan;

    if (band < 1 || band > gen6705Device.numBand || chan < 1 || chan > gen6705Device.numChan)
        return;

    gen6705SetFreq(vtx58FreqTable[band - 1][chan - 1]);

    gen6705Device.curBand = band;
    gen6705Device.curChan = chan;
}

bool gen6705GetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    *pBand = gen6705Device.curBand;
    *pChan = gen6705Device.curChan;

    return true;
}

void gen6705Init(void)
{
    vtxCommonRegisterDevice(&gen6705Device);
}

#if 0
// This one is from SIRINFPV
void gen6705SetRFPower(uint8_t reduce_power)
{
    rtc6705_write_register(7, (reduce_power ? (PA_CONTROL_DEFAULT | PD_Q5G_MASK) & (~(PA5G_PW_MASK | PA5G_BS_MASK)) : PA_CONTROL_DEFAULT));
}
#endif

static vtxVTable_t gen6705VTable = {
    .process = NULL,
    .getDeviceType = gen6705GetDeviceType,
    .isReady = gen6705IsReady,
    .setBandChan = gen6705SetBandChan,
    //.setPowerByIndex = gen6705SetPowerByIndex,
    //.setPitmode = gen6705SetPitmode,
    .getBandChan = gen6705GetBandChan,
    //.getPowerIndex = gen6705GetPowerIndex,
    //.getPitmode = gen6705GetPitmode,
};
#endif

