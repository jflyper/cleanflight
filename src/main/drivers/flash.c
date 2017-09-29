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

#include "build/debug.h"

//#ifdef USE_FLASH

#include "flash.h"
#include "flash_m25p16.h"
#include "flash_w25n01g.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

static busDevice_t busInstance;
static busDevice_t *busdev;

static const flashVTable_t *flashVTable = NULL;

// Read chip identification and send it to device detect

bool flashInit(const flashConfig_t *flashConfig)
{
    busdev = &busInstance;
    busdev->bustype = BUSTYPE_SPI;
    spiBusSetInstance(busdev, spiInstanceByDevice(SPI_CFG_TO_DEV(flashConfig->spiDevice)));
    if (flashConfig->csTag) {
        busdev->busdev_u.spi.csnPin = IOGetByTag(flashConfig->csTag);
    } else {
        return false;
    }

    IOInit(busdev->busdev_u.spi.csnPin, OWNER_FLASH_CS, 0);
    IOConfigGPIO(busdev->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(busdev->busdev_u.spi.csnPin);

#ifndef M25P16_SPI_SHARED
    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    //spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_FAST);
    spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD*2);
#endif

#ifdef USE_FLASH_M25P16
    if ((flashVTable = m25p16_detect(busdev))) {
        return true;
    }
#endif
#ifdef USE_FLASH_W25N01G
    if ((flashVTable = w25n01g_detect(busdev))) {
        return true;
    }
#endif

    return false;
}

#undef DEBUG_FLASH_NOFLASH_CALLER
#ifdef DEBUG_FLASH_NOFLASH_CALLER
#define NOFLASH_CALLER(idx, bit) { debug[idx] |= (1 << (bit)); }; struct dummy_s
#else
#define NOFLASH_CALLER(idx, bit)
#endif

bool flashIsReady(void)
{
    if (flashVTable && flashVTable->isReady) {
        return flashVTable->isReady();
    } else {
        NOFLASH_CALLER(3, 0);
        return false;
    }
}

bool flashWaitForReady(uint32_t timeoutMillis)
{
    if (flashVTable && flashVTable->waitForReady) {
        return flashVTable->waitForReady(timeoutMillis);
    } else {
        NOFLASH_CALLER(3, 1);
        return false;
    }
}

void flashEraseSector(uint32_t address)
{
    if (flashVTable && flashVTable->eraseSector) {
        flashVTable->eraseSector(address);
    } else {
        NOFLASH_CALLER(3, 2);
    }
}

void flashEraseCompletely(void)
{
    if (flashVTable && flashVTable->eraseCompletely) {
        flashVTable->eraseCompletely();
    } else {
        NOFLASH_CALLER(3, 3);
    }
}

void flashPageProgramBegin(uint32_t address)
{
    if (flashVTable && flashVTable->pageProgramBegin) {
        flashVTable->pageProgramBegin(address);
    } else {
        NOFLASH_CALLER(3, 4);
    }
}

void flashPageProgramContinue(const uint8_t *data, int length)
{
    if (flashVTable && flashVTable->pageProgramContinue) {
        flashVTable->pageProgramContinue(data, length);
    } else {
        NOFLASH_CALLER(3, 5);
    }
}

void flashPageProgramFinish(void)
{
    if (flashVTable && flashVTable->pageProgramFinish) {
        flashVTable->pageProgramFinish();
    } else {
        NOFLASH_CALLER(3, 6);
    }
}

void flashPageProgram(uint32_t address, const uint8_t *data, int length)
{
    if (flashVTable && flashVTable->pageProgram) {
        flashVTable->pageProgram(address, data, length);
    } else {
        NOFLASH_CALLER(3, 7);
    }
}

int flashReadBytes(uint32_t address, uint8_t *buffer, int length)
{
    if (flashVTable && flashVTable->readBytes) {
        return flashVTable->readBytes(address, buffer, length);
    } else {
        NOFLASH_CALLER(3, 8);
        return 0;
    }
}

void flashClose(void)
{
    if (flashVTable && flashVTable->close) {
        flashVTable->close();
    }
}

static flashGeometry_t noFlashGeometry = {
    .totalSize = 0,
};

const flashGeometry_t *flashGetGeometry(void)
{
    if (flashVTable && flashVTable->getGeometry) {
        return flashVTable->getGeometry();
    } else {
        return &noFlashGeometry;
    }
}
//#endif // USE_FLASH
