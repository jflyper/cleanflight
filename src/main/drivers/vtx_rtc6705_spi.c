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
 * Author: Giles Burgess (giles@multiflite.co.uk)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef VTX_RTC6705_SPI

#include "common/maths.h"

#include "io.h"
#include "bus_spi.h"
#include "system.h"
#include "vtx_gen6705.h"
#include "vtx_rtc6705_spi.h"

#define DISABLE_RTC6705 GPIO_SetBits(RTC6705_CS_GPIO,   RTC6705_CS_PIN)
#define ENABLE_RTC6705  GPIO_ResetBits(RTC6705_CS_GPIO, RTC6705_CS_PIN)

#if defined(SPRACINGF3NEO)
static IO_t vtxPowerPin        = IO_NONE;
#endif

#define ENABLE_VTX_POWER       IOLo(vtxPowerPin)
#define DISABLE_VTX_POWER      IOHi(vtxPowerPin)

static void rtc6705WriteRegister(uint8_t addr, uint32_t data); // forward

static gen6705Device_t device = {
    .writeRegister = rtc6705WriteRegister,
};

/**
 * Send a command and return if good
 * TODO chip detect
 */
static bool rtc6705IsReady(void)
{
    // Sleep a little bit to make sure it has booted
    delay(50);

    // TODO Do a read and get current config (note this would be reading on MOSI (data) line)

    return true;
}

/**
 * Reverse a uint32_t (LSB to MSB)
 * This is easier for when generating the frequency to then
 * reverse the bits afterwards
 */
static uint32_t reverse32(uint32_t in)
{
    uint32_t out = 0;

    for (uint8_t i = 0 ; i < 32 ; i++)
    {
        out |= ((in>>i) & 1)<<(31-i);
    }

    return out;
}

/**
 * Start chip if available
 */

// XXX Overhaul this when configurable SPI is ready.
bool rtc6705_spi_init()
{
// XXX Can't handle this atm.
// XXX RTC6705PinConfig with a couple of aux pin may work.
#ifdef RTC6705_POWER_PIN
    vtxPowerPin = IOGetByTag(IO_TAG(RTC6705_POWER_PIN));
    IOInit(vtxPowerPin, OWNER_VTX, 0);
    IOConfigGPIO(vtxPowerPin, IOCFG_OUT_PP);

    ENABLE_VTX_POWER;
#endif

    DISABLE_RTC6705;
    spiSetDivisor(RTC6705_SPI_INSTANCE, SPI_CLOCK_SLOW);

    gen6705RegisterDevice(&device);

    return rtc6705IsReady();
}

/**
 * Transfer a 25bit packet to RTC6705
 * This will just send it as a 32bit packet LSB meaning
 * extra 0's get truncated on RTC6705 end
 */
static void rtc6705Transfer(uint32_t command)
{
    command = reverse32(command);

    ENABLE_RTC6705;

    spiTransferByte(RTC6705_SPI_INSTANCE, (command >> 24) & 0xFF);
    spiTransferByte(RTC6705_SPI_INSTANCE, (command >> 16) & 0xFF);
    spiTransferByte(RTC6705_SPI_INSTANCE, (command >> 8) & 0xFF);
    spiTransferByte(RTC6705_SPI_INSTANCE, (command >> 0) & 0xFF);

    DISABLE_RTC6705;
}

static void rtc6705WriteRegister(uint8_t addr, uint32_t data)
{
    uint32_t command = (addr << 21)|(1 << 20)|(data & 0xfffff);

    rtc6705Transfer(command);
}
#endif
