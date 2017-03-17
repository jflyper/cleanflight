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

#ifdef VTX_RTC6705_SOFTSPI

#ifdef VTX_COMMON_DPRINTF
#include "io/serial.h"
extern serialPort_t *debugSerialPort;
# define dprintf(x) if (debugSerialPort) printf x
#else // VTX_COMMON_DPRINTF
# define dprintf(x)
#endif // VTX_COMMON_DPRINTF

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/bus_spi_soft.h"
#include "drivers/bus_spi.h"
#include "drivers/vtx_gen6705.h"
#include "drivers/vtx_rtc6705_softspi.h"

#define RTC6705_SPICLK_ON     IOHi(rtc6705ClkPin)
#define RTC6705_SPICLK_OFF    IOLo(rtc6705ClkPin)

#define RTC6705_SPIDATA_ON    IOHi(rtc6705DataPin)
#define RTC6705_SPIDATA_OFF   IOLo(rtc6705DataPin)

#define RTC6705_SPILE_ON      IOHi(rtc6705LePin)
#define RTC6705_SPILE_OFF     IOLo(rtc6705LePin)

static IO_t rtc6705DataPin = IO_NONE;
static IO_t rtc6705LePin = IO_NONE;
static IO_t rtc6705ClkPin = IO_NONE;

static void rtc6705_write_register(uint8_t addr, uint32_t data); // forward

static gen6705Device_t device = {
    .writeRegister = rtc6705_write_register,
};

void rtc6705_softspi_pinConfigReset(SPIPinConfig_t *pSPIPinConfig)
{
    pSPIPinConfig->nssTag = IO_TAG(RTC6705_SPILE_PIN);
    pSPIPinConfig->sckTag = IO_TAG(RTC6705_SPICLK_PIN);
    pSPIPinConfig->mosiTag = IO_TAG(RTC6705_SPIDATA_PIN);
    pSPIPinConfig->misoTag = IO_TAG_NONE;
}

bool rtc6705_softspi_init(SPIPinConfig_t *pSPIPinConfig)
{
    dprintf(("rtc6705_softspi_init: top\r\n"));

    rtc6705DataPin = IOGetByTag(pSPIPinConfig->mosiTag);
    rtc6705LePin   = IOGetByTag(pSPIPinConfig->nssTag);
    rtc6705ClkPin  = IOGetByTag(pSPIPinConfig->sckTag);

    if (!(rtc6705DataPin && rtc6705LePin && rtc6705ClkPin))
        return false;

    dprintf(("rtc6705_softspi_init: pin ok\r\n"));

    IOInit(rtc6705DataPin, OWNER_SPI_MOSI, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705DataPin, IOCFG_OUT_PP);

    IOInit(rtc6705LePin, OWNER_SPI_CS, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705LePin, IOCFG_OUT_PP);

    IOInit(rtc6705ClkPin, OWNER_SPI_SCK, RESOURCE_SOFT_OFFSET);
    IOConfigGPIO(rtc6705ClkPin, IOCFG_OUT_PP);

    // XXX Should power down the pre-driver (PD_Q5G = 1)?

    gen6705RegisterDevice(&device);

    return true;
}

static void rtc6705_write_register(uint8_t addr, uint32_t data)
{
#if 0
debug[0] = addr;
debug[1] = (int16_t)(data >> 16);
debug[2] = (int16_t)data;
#endif

    uint8_t i;

    RTC6705_SPILE_OFF;
    delayMicroseconds(1);

    // send address
    for (i=0; i<4; i++) {
        if ((addr >> i) & 1)
            RTC6705_SPIDATA_ON;
        else
            RTC6705_SPIDATA_OFF;

        RTC6705_SPICLK_ON;
        delayMicroseconds(1);
        RTC6705_SPICLK_OFF;
        delayMicroseconds(1);
    }

    // send write bit
    RTC6705_SPIDATA_ON;
    delayMicroseconds(1);
    RTC6705_SPICLK_ON;
    delayMicroseconds(1);
    RTC6705_SPICLK_OFF;

    // send register bits
    delayMicroseconds(1);
    for (i=0; i<20; i++) {
        if ((data >> i) & 1)
            RTC6705_SPIDATA_ON;
        else
            RTC6705_SPIDATA_OFF;
        RTC6705_SPICLK_ON;
        delayMicroseconds(1);
        RTC6705_SPICLK_OFF;
        delayMicroseconds(1);
    }

    RTC6705_SPILE_ON;
}
#endif
