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
gen6705 provides an 'intelligent' VTX with memories about modes,
band, channel and transmitting power.
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef VTX_RTC6705

#include "build/debug.h"
#include "drivers/vtx_debug.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "drivers/vtx_common.h"
#include "io/vtx_gen6705.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"

static gen6705Device_t *pDevice = NULL;
static vtxConfig_t *pVtxConfig = NULL;

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

void gen6705SetFselMode(uint8_t mode)
{
    dprintf(("gen6705SetFselMode: mode %d\r\n", mode));

    pVtxConfig->vtx_mode = mode;
}

static void gen6705SetFreqRegisters(uint16_t freq)
{
    dprintf(("gen6705SetFreqRegisters: freq %d\r\n", freq));

    uint32_t wfreq = (uint32_t)freq * 1000;
    uint32_t N, A;

    wfreq /= 40;
    N = wfreq / 64;
    A = wfreq % 64;
    gen6705WriteRegister(0, 400);
    gen6705WriteRegister(1, (N << 7) | A);
}

static void _gen6705SetFreq(uint16_t freq)
{
    dprintf(("_gen6705SetFreq: freq %d\r\n", freq));

    gen6705SetFreqRegisters(freq);
}

void gen6705SetFreq(uint16_t freq)
{
    _gen6705SetFreq(freq);

    pVtxConfig->vtx_mhz = freq;
}

static void _gen6705SetBandChan(uint8_t band, uint8_t chan)
{
    dprintf(("_gen6705SetBandChan: band %d chan %d\r\n", band, chan));

    if (band < 1 || band > gen6705Device.numBand || chan < 1 || chan > gen6705Device.numChan)
        return;

    gen6705SetFreqRegisters(vtx58FreqTable[band - 1][chan - 1]);

}

void gen6705SetBandChan(uint8_t band, uint8_t chan)
{
    _gen6705SetBandChan(band, chan);

    pVtxConfig->vtx_band = band;
    pVtxConfig->vtx_channel = chan;
}

bool gen6705GetBandChan(uint8_t *pBand, uint8_t *pChan)
{
    *pBand = pVtxConfig->vtx_band;
    *pChan = pVtxConfig->vtx_channel;

    return true;
}

bool gen6705GetFreq(uint16_t *pFreq)
{
    *pFreq = pVtxConfig->vtx_mhz;

    return true;
}

bool gen6705GetFselMode(uint8_t *pMode)
{
    *pMode = pVtxConfig->vtx_mode;

    return true;
}

void gen6705Init(vtxConfig_t *pVtxConfigToUse)
{
    if (!pDevice)
        return;

    pVtxConfig = pVtxConfigToUse;

    // Initialize per current configuration
    // XXX Take care the power/pit mode

    switch (pVtxConfig->vtx_mode) {
    case 0: // band/chan
        _gen6705SetBandChan(pVtxConfig->vtx_band, pVtxConfig->vtx_channel);
        break;

    case 1: // direct
        _gen6705SetFreq(pVtxConfig->vtx_mhz);
        break;

    case 2: // vtxrc
        // XXX What is the default for VTXRC???
        break;
    }

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
    .setFreq = gen6705SetFreq,
    //.setPowerByIndex = gen6705SetPowerByIndex,
    .setFselMode = gen6705SetFselMode,
    //.setPitmode = gen6705SetPitmode,
    .getBandChan = gen6705GetBandChan,
    .getFreq = gen6705GetFreq,
    //.getPowerIndex = gen6705GetPowerIndex,
    .getFselMode = gen6705GetFselMode,
    //.getPitmode = gen6705GetPitmode,
};
#endif

