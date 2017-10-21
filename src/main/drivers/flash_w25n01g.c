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
 *
 * Author: jflyper
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_W25N01G

#include "flash.h"
#include "flash_w25n01g.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#define FLASH_W25N01G_DPRINTF

#ifdef FLASH_W25N01G_DPRINTF
#include "common/printf.h"
#include "common/utils.h"
#include "io/serial.h"
serialPort_t *debugSerialPort = NULL;
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
#define DPRINTF(x) tfp_printf x
#else
#define DPRINTF(x)
#endif

// JEDEC ID
#define JEDEC_ID_WINBOND_W25N01GV    0xEFAA21
#define JEDEC_ID_WINBOND_W25M02GV    0xEFAB21

// Device size parameters
#define W25N01G_PAGE_SIZE         2048
#define W25N01G_PAGES_PER_BLOCK   64
#define W25N01G_BLOCKS_PER_DIE 1024

// Instructions

#define W25N01G_INSTRUCTION_RDID             0x9F
#define W25N01G_INSTRUCTION_DEVICE_RESET     0xFF
#define W25N01G_INSTRUCTION_READ_STATUS_REG  0x05
#define W25N01G_INSTRUCTION_WRITE_STATUS_REG 0x01
#define W25N01G_INSTRUCTION_WRITE_ENABLE     0x06
#define W25N01G_INSTRUCTION_DIE_SELECT       0xC2
#define W25N01G_INSTRUCTION_BLOCK_ERASE      0xD8
#define W25N01G_INSTRUCTION_READ_BBM_LUT     0xA5
#define W25N01G_INSTRUCTION_BB_MANAGEMENT    0xA1
#define W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD        0x02
#define W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD 0x84
#define W25N01G_INSTRUCTION_PROGRAM_EXECUTE  0x10
#define W25N01G_INSTRUCTION_PAGE_DATA_READ   0x13
#define W25N01G_INSTRUCTION_READ_DATA        0x03
#define W25N01G_INSTRUCTION_FAST_READ        0x1B

// Configu/status register addresses
#define W25N01G_PROT_REG 0xA0
#define W25N01G_CONF_REG 0xB0
#define W25N01G_STAT_REG 0xC0

// Bits in config/status register 2 (W25N01G_CONF_REG)
#define W25N01G_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE   (1 << 3)

// Bits in config/status register 3 (W25N01G_STATREG)
#define W25N01G_STATUS_BBM_LUT_FULL       (1 << 6)
#define W25N01G_STATUS_FLAG_ECC_POS       4
#define W25N01G_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N01G_STATUS_FLAG_ECC(status)   (((status) & W25N01G_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N01G_STATUS_PROGRAM_FAIL       (1 << 3)
#define W25N01G_STATUS_ERASE_FAIL         (1 << 2)
#define W25N01G_STATUS_FLAG_WRITE_ENABLED (1 << 1)
#define W25N01G_STATUS_FLAG_BUSY          (1 << 0)

// Bits in LBA for BB LUT
#define W25N01G_BBLUT_STATUS_ENABLED (1 << 15)
#define W25N01G_BBLUT_STATUS_INVALID (1 << 14)
#define W25N01G_BBLUT_STATUS_MASK    (W25N01G_BBLUT_STATUS_ENABLED | W25N01G_BBLUT_STATUS_INVALID)

// Some useful defs and macros
#define W25N01G_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_PAGE(laddr) ((laddr) / W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_BLOCK(laddr) (W25N01G_LINEAR_TO_PAGE(laddr) / W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_PAGE(block) ((block) * W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_LINEAR(block) (W25N01G_BLOCK_TO_PAGE(block) * W25N01G_PAGE_SIZE)

// BB replacement area
#define W25N01G_BB_MARKER_BLOCKS           1
#define W25N01G_BB_REPLACEMENT_BLOCKS      21
#define W25N01G_BB_REPLACEMENT_START_BLOCK (W25N01G_BLOCKS_PER_DIE - W25N01G_BB_REPLACEMENT_BLOCKS)
#define W25N01G_BB_MARKER_BLOCK            (W25N01G_BB_REPLACEMENT_START_BLOCK - W25N01G_BB_MARKER_BLOCKS)

// XXX These will be gone
#define DISABLE_M25P16       IOHi(bus->busdev_u.spi.csnPin); __NOP()
#define ENABLE_M25P16        __NOP(); IOLo(bus->busdev_u.spi.csnPin)

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N01G_TIMEOUT_PAGE_READ_MS      2   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_MS   2   // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS   15   // tBEmax = 10ms

#define W25N01G_MAX_DIES    4

static busDevice_t *bus;

typedef struct bblut_s {
    uint16_t pba;
    uint16_t lba;
} bblut_t;

typedef struct flashDie_s {
    bool couldBeBusy;
} flashDie_t;

typedef struct flashDevice_s {
    uint8_t dieCount;
    uint8_t currentDie;
    flashDie_t dies[W25N01G_MAX_DIES];
} flashDevice_t;

flashDevice_t flashDevice = {
    .currentDie = 0,
};

static flashGeometry_t geometry = {
    .sectors = 1024,         // Blocks
    .pagesPerSector = 64,    // Pages/Blocks
    .pageSize = 2048,
};

/*
 * Whether we've performed an action that could have made the device busy for writes.
 *
 * This allows us to avoid polling for writable status when it is definitely ready already.
 */
static bool couldBeBusy = false;

/**
 * Send the given command byte to the device.
 */
static void w25n01g_performOneByteCommand(uint8_t command)
{
    ENABLE_M25P16;
    spiTransferByte(bus->busdev_u.spi.instance, command);
    DISABLE_M25P16;
}

static uint8_t w25n01g_readRegister(uint8_t reg)
{
    const uint8_t cmd[3] = { W25N01G_INSTRUCTION_READ_STATUS_REG, reg, 0 };
    uint8_t in[3];

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, in, sizeof(cmd));
    DISABLE_M25P16;

    return in[2];
}

static void w25n01g_writeRegister(uint8_t reg, uint8_t data)
{
    const uint8_t cmd[3] = { W25N01G_INSTRUCTION_WRITE_STATUS_REG, reg, data };

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE_M25P16;
}

static void w25n01g_deviceReset(void)
{
    w25n01g_performOneByteCommand(W25N01G_INSTRUCTION_DEVICE_RESET);

    // Protection to upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on
    //w25n01g_writeRegister(W25N01G_PROT_REG, (5 << 3)|(0 << 2)|(1 << 1));

    // No protection, WP-E on
    w25n01g_writeRegister(W25N01G_PROT_REG, (0 << 3)|(0 << 2)|(1 << 1));

    // Continuous mode (BUF = 0), ECC enabled (ECC = 1)
    w25n01g_writeRegister(W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE);
}

static void w25n01g_readUUID(void)
{
}

bool w25n01g_isReady(void)
{
#if 0
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    couldBeBusy = couldBeBusy && ((w25n01g_readRegister(W25N01G_STAT_REG) & W25N01G_STATUS_FLAG_BUSY) != 0);

    return !couldBeBusy;
#else
    uint8_t status = w25n01g_readRegister(W25N01G_STAT_REG);

    if (status & W25N01G_STATUS_PROGRAM_FAIL) {
        DPRINTF(("*** PROGRAM_FAIL\r\n"));
    }

    if (status & W25N01G_STATUS_ERASE_FAIL) {
        DPRINTF(("*** ERASE_FAIL\r\n"));
    }

    uint8_t eccCode;
    if ((eccCode = W25N01G_STATUS_FLAG_ECC(status))) {
        DPRINTF(("*** ECC %x\r\n", eccCode));
    }

    return ((status & W25N01G_STATUS_FLAG_BUSY) == 0);
#endif
}

bool w25n01g_waitForReady(uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!w25n01g_isReady()) {
        if (millis() - time > timeoutMillis) {
            DPRINTF(("*** TIMEOUT %d\r\n", timeoutMillis));
            return false;
        }
    }

    return true;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
static void w25n01g_writeEnable(void)
{
    w25n01g_performOneByteCommand(W25N01G_INSTRUCTION_WRITE_ENABLE);

    // Assume that we're about to do some writing, so the device is just about to become busy
    couldBeBusy = true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
const flashVTable_t w25n01g_vTable;

void w25n01g_deviceInit(void);

const flashVTable_t *w25n01g_detect(busDevice_t *busdev)
{
#ifdef FLASH_W25N01G_DPRINTF
    // Setup debugSerialPort
    debugSerialPort = openSerialPort(DPRINTF_SERIAL_PORT, FUNCTION_NONE, NULL, 115200, MODE_RXTX, 0);

    if (debugSerialPort) {
        setPrintfSerialPort(debugSerialPort);
        DPRINTF(("debug print init: OK\r\n"));
    }
#endif

    const uint8_t out[] = { W25N01G_INSTRUCTION_RDID, 0, 0, 0, 0 };

    delay(50); // short delay required after initialisation of SPI device instance.

    bus = busdev;

    /* Just in case transfer fails and writes nothing, so we don't try to verify the ID against random garbage
     * from the stack:
     */
    uint8_t in[5];
    in[2] = 0;

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, out, in, sizeof(out));
    DISABLE_M25P16;

    // Manufacturer, memory type, and capacity
    const uint32_t chipID = (in[2] << 16) | (in[3] << 8) | (in[4]);

    switch (chipID) {
    case JEDEC_ID_WINBOND_W25N01GV:
        geometry.dieCount = 1;
        break;
    case JEDEC_ID_WINBOND_W25M02GV:
        geometry.dieCount = 2;
        break;

    default:
        // Unsupported chip
        geometry.sectors = 0;
        geometry.pagesPerSector = 0;

        geometry.sectorSize = 0;
        geometry.totalSize = 0;
        return NULL;
    }

    geometry.sectors -= W25N01G_BB_REPLACEMENT_BLOCKS;

    geometry.sectorSize = geometry.pagesPerSector * geometry.pageSize;
    geometry.totalSize = geometry.sectorSize * geometry.sectors;

    couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    w25n01g_deviceReset();

    // Upper 4MB (32 blocks * 128KB/block) will be used for bad block replacement area.

    // We will protect this area from direct write by hardware protection facility
    // (See 7.1.1 Block Protect Bits).
    // XXX Does this work???

    // Blocks in this area are only written through bad block LUT,
    // and factory written bad block marker in unused blocks are retained.

    // When a replacement block is required,
    // (1) "Read BB LUT" command is used to obtain the last block mapped,
    // (2) blocks after the last block is scanned for a good block,
    // (3) the first good block is used for replacement, and the BB LUT is updated.

    // There are only 20 BB LUT entries, and there are 32 replacement blocks.
    // There will be a least chance of running out of replacement blocks.
    // If it ever run out, the device becomes unusable.


#if 0
    w25n01g_writeEnable();

    uint8_t sr1, sr2, sr3;
    sr1 = w25n01g_readRegister(W25N01G_PROT_REG);
    sr2 = w25n01g_readRegister(W25N01G_CONF_REG);
    sr3 = w25n01g_readRegister(W25N01G_STAT_REG);

    debug[1] = sr1;
    debug[2] = sr2;
    debug[3] = sr3;
#endif

#if 0
    bblut_t bblut[20];
    w25n01g_read_bblut(bblut);
#endif

    w25n01g_deviceInit();

    return &w25n01g_vTable;
}

/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
void w25n01g_eraseSector(uint32_t address)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_BLOCK_ERASE, 0, W25N01G_LINEAR_TO_PAGE(address) >> 8, W25N01G_LINEAR_TO_PAGE(address) & 0xff };

    w25n01g_waitForReady(W25N01G_TIMEOUT_BLOCK_ERASE_MS);

    w25n01g_writeEnable();

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE_M25P16;
}

//
// W25N01G does not support full chip erase.
// Call eraseSector repeatedly.

void w25n01g_eraseCompletely(void)
{
    for (uint32_t block = 0; block < geometry.sectors; block++) {
        for (int die = 0; die < geometry.dieCount; die++) {
            // Select die
            //w25n01g_selectDie(die);

            // Wait for this die to become ready
            w25n01g_waitForReady(W25N01G_TIMEOUT_BLOCK_ERASE_MS);

            // Issue erase block command
            w25n01g_writeEnable();
            w25n01g_eraseSector(W25N01G_BLOCK_TO_LINEAR(block));
        }
    }
}

static void w25n01g_programDataLoad(uint16_t columnAddress, const uint8_t *data, int length)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress& 0xff };

    //DPRINTF(("    load WaitForReady\r\n"));
    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    load Issuing command\r\n"));
    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    spiTransfer(bus->busdev_u.spi.instance, data, NULL, length);
    DISABLE_M25P16;
    //DPRINTF(("    load Done\r\n"));
}

static void w25n01g_randomProgramDataLoad(uint16_t columnAddress, const uint8_t *data, int length)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress& 0xff };

    //DPRINTF(("    random WaitForReady\r\n"));
    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    random Issuing command\r\n"));
    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    spiTransfer(bus->busdev_u.spi.instance, data, NULL, length);
    DISABLE_M25P16;
    //DPRINTF(("    random Done\r\n"));
}

static void w25n01g_programExecute(uint32_t pageAddress)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_EXECUTE, 0, pageAddress >> 8, pageAddress & 0xff };

    //DPRINTF(("    execute WaitForReady\r\n"));
    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    //DPRINTF(("    execute Issueing command\r\n"));
    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE_M25P16;
    //DPRINTF(("    execute Done\r\n"));
}

//
// Writes are done in three steps:
// (1) Load internal data buffer with data to write
//     - We use "Random Load Program Data", as "Load Program Data" resets unused data bytes in the buffer to 0xff.
//     - Each "Random Load Program Data" instruction must be accompanied by at least a single data.
//     - Each "Random Load Program Data" instruction terminates at the rising of CS.
// (2) Enable write
// (3) Issue "Execute Program"
//

/*
flashfs page program behavior
- Single program never crosses page boundary.
- Except for this characteristic, it program arbitral size.
- Write address is, naturally, not a page boundary.

To cope with this behavior.

pageProgramBegin:
If buffer is dirty and programLoadAddress != address, then the last page is a partial write;
issue PAGE_PROGRAM_EXECUTE to flash buffer contents, clear dirty and record the address as programLoadAddress and programStartAddress.
Else do nothing.

pageProgramContinue:
Mark buffer as dirty.
If programLoadAddress is on page boundary, then issue PROGRAM_LOAD_DATA, else issue RANDOM_PROGRAM_LOAD_DATA.
Update programLoadAddress.
Optionally observe the programLoadAddress, and if it's on page boundary, issue PAGE_PROGRAM_EXECUTE.

pageProgramFinish:
Observe programLoadAddress. If it's on page boundary, issue PAGE_PROGRAM_EXECUTE and clear dirty, else just return.
If pageProgramContinue observes the page boundary, then do nothing(?).
*/

static uint32_t programStartAddress;
static uint32_t programLoadAddress;
bool bufferDirty = false;
bool isProgramming = false;

#define DEBUG_PAGE_PROGRAM

void w25n01g_pageProgramBegin(uint32_t address)
{
    DPRINTF(("pageProgramBegin: address 0x%x\r\n", address));

    if (bufferDirty) {
        if (address != programLoadAddress) {
            DPRINTF(("    Buffer dirty, flushing\r\n"));
            DPRINTF(("    Wait for ready\r\n"));
            w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

            isProgramming = false;

            DPRINTF(("    Write enable\r\n"));
            w25n01g_writeEnable();

            DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));
            w25n01g_programExecute(W25N01G_LINEAR_TO_PAGE(programStartAddress));

            bufferDirty = false;
            isProgramming = true;
        } else {
            DPRINTF(("    Continuation\r\n"));
        }
    } else {
        DPRINTF(("    Fresh page\r\n"));
        programStartAddress = programLoadAddress = address;
    }
}

void w25n01g_pageProgramContinue(const uint8_t *data, int length)
{
    DPRINTF(("pageProgramContinue: length 0x%x (programLoadAddress 0x%x)\r\n", length, programLoadAddress));

    DPRINTF(("    Wait for ready\r\n"));
    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    DPRINTF(("    Write enable\r\n"));
    w25n01g_writeEnable();

    isProgramming = false;

    if (!bufferDirty) {
        DPRINTF(("    DATA_LOAD CA 0x%x length 0x%x\r\n", W25N01G_LINEAR_TO_COLUMN(programLoadAddress), length));
        w25n01g_programDataLoad(W25N01G_LINEAR_TO_COLUMN(programLoadAddress), data, length);
    } else {
        DPRINTF(("    RANDOM_DATA_LOAD CA 0x%x length 0x%x\r\n", W25N01G_LINEAR_TO_COLUMN(programLoadAddress), length));
        w25n01g_randomProgramDataLoad(W25N01G_LINEAR_TO_COLUMN(programLoadAddress), data, length);
    }

    // XXX Test if write enable is reset after each data loading.

    bufferDirty = true;
    programLoadAddress += length;
}

void w25n01g_pageProgramFinish(void)
{
    DPRINTF(("pageProgramFinish: (loaded 0x%x bytes)\r\n", programLoadAddress - programStartAddress));

    if (!bufferDirty || W25N01G_LINEAR_TO_COLUMN(programLoadAddress) != 0) {
        DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));
        w25n01g_programExecute(W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;
    } else {
        DPRINTF(("    Ignoring\r\n"));
    }
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
void w25n01g_pageProgram(uint32_t address, const uint8_t *data, int length)
{
    w25n01g_pageProgramBegin(address);
    w25n01g_pageProgramContinue(data, length);
    w25n01g_pageProgramFinish();
}

void w25n01g_close(void)
{
    DPRINTF(("close: (loaded 0x%x bytes)\r\n", programLoadAddress - programStartAddress));

    if (bufferDirty) {
        DPRINTF(("    PROGRAM_EXECUTE PA 0x%x\r\n", W25N01G_LINEAR_TO_PAGE(programStartAddress)));
        w25n01g_programExecute(W25N01G_LINEAR_TO_PAGE(programStartAddress));

        bufferDirty = false;
        isProgramming = true;
    } else {
        DPRINTF(("    Page is clean\r\n"));
        isProgramming = false;
    }
}

void w25n01g_addError(uint32_t address, uint8_t code)
{
    DPRINTF(("addError: PA %x BA %x code %d\r\n", W25N01G_LINEAR_TO_PAGE(address), W25N01G_LINEAR_TO_BLOCK(address), code));
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to W25N01G_TIMEOUT_PAGE_READ_MS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */

// Continuous read mode (BUF = 0) will be used.
// (1) "Page Data Read" command is executed for the page pointed by address
// (2) "Read Data" command is executed for bytes not requested and data are discarded
// (3) "Read Data" command is executed and data are stored directly into caller's buffer

static uint32_t currentReadAddress = UINT32_MAX; // XXX Per die data

//#define READBYTES_DPRINTF DPRINTF
#define READBYTES_DPRINTF(x)

int w25n01g_readBytes(uint32_t address, uint8_t *buffer, int length)
{
    uint8_t cmd[4];

    READBYTES_DPRINTF(("readBytes: address 0x%x length %d\r\n", address, length));

    if (address != currentReadAddress) {
        cmd[0] = W25N01G_INSTRUCTION_PAGE_DATA_READ;
        cmd[1] = 0;
        cmd[2] = W25N01G_LINEAR_TO_PAGE(address) >> 8;
        cmd[3] = W25N01G_LINEAR_TO_PAGE(address);

        if (!w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_READ_MS)) {
            return 0;
        }

        READBYTES_DPRINTF(("readBytes: PAGE_DATA_READ %02x %02x\r\n", cmd[2], cmd[3]));

        ENABLE_M25P16;
        spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 4);
        DISABLE_M25P16;
    }

    cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;

    int remainingLength = length;
    uint16_t transferLength;
    uint16_t column = W25N01G_LINEAR_TO_COLUMN(address);

    if (column) {
        READBYTES_DPRINTF(("readBytes: READ_DATA (skipping)\r\n"));

        if (!w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_READ_MS)) {
            return 0;
        }

        ENABLE_M25P16;
        spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 4);
        DISABLE_M25P16;

        transferLength = W25N01G_PAGE_SIZE - column;
    } else {
        transferLength = (length < W25N01G_PAGE_SIZE) ? length : W25N01G_PAGE_SIZE;
    }

    while (transferLength) {

        READBYTES_DPRINTF(("readBytes: READ_DATA transferLength 0x%x\r\n", transferLength));

        if (!w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_READ_MS)) {
            return 0;
        }

        ENABLE_M25P16;
        spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 4);
        spiTransfer(bus->busdev_u.spi.instance, NULL, buffer, transferLength);
        DISABLE_M25P16;

        // Check ECC

        uint8_t statReg = w25n01g_readRegister(W25N01G_STAT_REG);
        uint8_t eccCode = W25N01G_STATUS_FLAG_ECC(statReg);

        switch (eccCode) {
        case 0: // Successful read, no ECC correction
            break;
        case 1: // Successful read with ECC correction
        case 2: // Uncorrectable ECC in a single page
        case 3: // Uncorrectable ECC in multiple pages
            w25n01g_addError(address, eccCode);
            w25n01g_deviceReset();
            break;
        }

        address += transferLength;
        buffer += transferLength;
        remainingLength -= transferLength;

        transferLength = (remainingLength < W25N01G_PAGE_SIZE) ? remainingLength : W25N01G_PAGE_SIZE;
    }

    currentReadAddress = address;

    return length - remainingLength;
}

int w25n01g_readExtensionBytes(uint32_t address, uint8_t *buffer, int length)
{
    uint8_t cmd[4];

    cmd[0] = W25N01G_INSTRUCTION_PAGE_DATA_READ;
    cmd[1] = 0;
    cmd[2] = W25N01G_LINEAR_TO_PAGE(address) >> 8;
    cmd[3] = W25N01G_LINEAR_TO_PAGE(address);

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 4);
    DISABLE_M25P16;

    if (!w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_READ_MS)) {
        return 0;
    }

    cmd[0] = W25N01G_INSTRUCTION_READ_DATA;
    cmd[1] = 0;
    cmd[2] = 2048 >> 8;
    cmd[3] = 2048;

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 4);
    spiTransfer(bus->busdev_u.spi.instance, NULL, buffer, length);
    DISABLE_M25P16;

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling w25n01g_init() (the result would have totalSize = 0).
 */
const flashGeometry_t* w25n01g_getGeometry(void)
{
    return &geometry;
}

const flashVTable_t w25n01g_vTable = {
    .isReady = w25n01g_isReady,
    .waitForReady = w25n01g_waitForReady,
    .eraseSector = w25n01g_eraseSector,
    .eraseCompletely = w25n01g_eraseCompletely,
    .pageProgramBegin = w25n01g_pageProgramBegin,
    .pageProgramContinue = w25n01g_pageProgramContinue,
    .pageProgramFinish = w25n01g_pageProgramFinish,
    .pageProgram = w25n01g_pageProgram,
    .close = w25n01g_close,
    .readBytes = w25n01g_readBytes,
    .getGeometry = w25n01g_getGeometry,
};

static void w25n01g_readBBLUT(bblut_t *bblut, int lutsize)
{
    uint8_t cmd[4];
    uint8_t in[4];

    cmd[0] = W25N01G_INSTRUCTION_READ_BBM_LUT;
    cmd[1] = 0;

    ENABLE_M25P16;

    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, 2);

    for (int i = 0 ; i < lutsize ; i++) {
        spiTransfer(bus->busdev_u.spi.instance, NULL, in, 4);
        bblut[i].pba = (in[0] << 16)|in[1];
        bblut[i].lba = (in[2] << 16)|in[3];
    }

    DISABLE_M25P16;
}

static void w25n01g_writeBBLUT(uint16_t lba, uint16_t pba)
{
    uint8_t cmd[5] = { W25N01G_INSTRUCTION_BB_MANAGEMENT, lba >> 8, lba, pba >> 8, pba };

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE_M25P16;

    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);
}

#define TOTAL_PAGES (W25N01G_PAGES_PER_BLOCK * W25N01G_BLOCKS_PER_DIE)

void myRandomInit(void);
uint32_t myRandom(void);

void w25n01g_deviceInit(void)
{
    // Read and print BBLUT


    bblut_t bblut[20];

    DPRINTF(("BBLUT:\r\n"));

    w25n01g_readBBLUT(bblut, 20);
    for (int i = 0 ; i < 20 ; i++) {
        if ((bblut[i].pba & W25N01G_BBLUT_STATUS_MASK) == W25N01G_BBLUT_STATUS_ENABLED) {
            DPRINTF(("%2d: %x:%x ", i, bblut[i].pba, bblut[i].lba));
            if ((bblut[i].pba & W25N01G_BBLUT_STATUS_MASK) == W25N01G_BBLUT_STATUS_INVALID) {
                DPRINTF(("invalid\r\n"));
            } else {
                DPRINTF(("valid\r\n"));
            }
        }
    }



#if 0
    // Test #1
    // Scan for initial factory recorded bad blocks

    DPRINTF(("scanning for bad block\r\n"));

    for (int block = 0 ; block < W25N01G_BLOCKS_PER_DIE ; block++) {
        uint8_t marker;

        w25n01g_readBytes(W25N01G_BLOCK_TO_LINEAR(block), &marker, 1);

        if (marker == 0xff) {
            continue;
        }

        if (++olines <= 20) {
            DPRINTF(("bad block at %d (marker 0x%x)\r\n", block, marker));
        }
    }
#endif

#if 0
    // Test #2
    // Write a page, read it back, erase it, confirm erase

    DPRINTF(("Writing\r\n"));
    uint32_t pageId;
    pageId = 0x12345678;
    w25n01g_pageProgram(0, (uint8_t *)&pageId, sizeof(pageId));

    DPRINTF(("Reading\r\n"));
    uint32_t readId;
    w25n01g_readBytes(0, (uint8_t *)&readId, sizeof(readId));

    if (readId != pageId) {
        DPRINTF(("readId != pageId (expected %x got %x)\r\n", pageId, readId));
    }

    DPRINTF(("Erasing\r\n"));
    w25n01g_eraseSector(0);
    w25n01g_waitForReady(W25N01G_TIMEOUT_BLOCK_ERASE_MS);

    DPRINTF(("Reading\r\n"));
    w25n01g_readBytes(0, (uint8_t *)&readId, sizeof(readId));

    if (readId != 0xffffffff) {
        DPRINTF(("readId != 0xffffffff (got %x)\r\n", readId));
    }
#endif

#if 0
    // Test #3
    // Random number generator test
    //myRandomTest();
    myRandomInit();
    for (int i = 0 ; i < 10 ; i++) {
        DPRINTF(("random %d\r\n", myRandom()));
    }
#endif

#if 0
    // Test #4
    // Write few pages with their page numbers.
    // Read entire device and see if non-empty page has its page number written.
#define PAGES_TO_TEST 100

    DPRINTF(("*** Testing random read/write\r\n"));

    // Verify clean device

    DPRINTF(("Scanning and erasing non-empty pages\r\n"));

    for (uint32_t page = 0 ; page < TOTAL_PAGES ; page++) {
        uint32_t pageID;

        w25n01g_readBytes(page * W25N01G_PAGE_SIZE, (uint8_t *)&pageID, sizeof(pageID));

        if (pageID == 0xffffffff) {
            continue;
        }

        DPRINTF(("Non-empty page: 0x%x\r\n", page));

        w25n01g_eraseSector(page * W25N01G_PAGE_SIZE);
    }

    myRandomInit();

    DPRINTF(("Writing\r\n"));

    for (int count = 0 ; count < PAGES_TO_TEST ; count++) {
        uint32_t pageID;
        pageID = myRandom() & (TOTAL_PAGES-1);

        DPRINTF(("Writing page 0x%x\r\n", pageID));

        w25n01g_pageProgram(pageID * W25N01G_PAGE_SIZE, (uint8_t *)&pageID, sizeof(pageID));
    }

    int nonEmpty = 0;

    DPRINTF(("Reading\r\n"));

    for (uint32_t page = 0 ; page < TOTAL_PAGES ; page++) {
        uint32_t pageID;

        w25n01g_readBytes(page * W25N01G_PAGE_SIZE, (uint8_t *)&pageID, sizeof(pageID));

        if (pageID == 0xffffffff) {
            continue;
        }

        ++nonEmpty;

        if (pageID == page) {
            continue;
        } else {
            DPRINTF(("Random r/w: id mismatch at page 0x%x (got pageID 0x%x)\r\n", page, pageID));
        }
    }

    DPRINTF(("Test pages %d, found %d non-empty pages\r\n", PAGES_TO_TEST, nonEmpty));

    DPRINTF(("Erasing device\r\n"));

    uint32_t eraseStartMs = millis();
    w25n01g_eraseCompletely();
    uint32_t eraseEndMs = millis();

    DPRINTF(("Erased in %d ms\r\n", eraseEndMs - eraseStartMs));
#endif

#if 0
    // TEST #
    // Scan a device for bad block by writing/reading
    uint8_t buffer[2048];

    DPRINTF(("Complete read/write\r\n"));

    w25n01g_eraseCompletely();

    for (int page = 0 ; page < TOTAL_PAGES ; page++) {
        buffer[0] = page >> 24;
        buffer[1] = page >> 16;
        buffer[2] = page >> 8;
        buffer[3] = page;

        w25n01g_pageProgram(page * W25N01G_PAGE_SIZE, buffer, 2048);
        w25n01g_readBytes(page * W25N01G_PAGE_SIZE, buffer, 2048);
    }

    DPRINTF(("Done\r\n"));
#endif

#if 0
    // TEST #
    // Full device read (time measurement)
    uint8_t buffer[2048];

    DPRINTF(("Full device read\r\n"));

    uint32_t startTimeMs = millis();

    for (int page = 0 ; page < TOTAL_PAGES ; page++) {
        w25n01g_readBytes(page * W25N01G_PAGE_SIZE, buffer, 2048);
    }

    uint32_t endTimeMs = millis();

    DPRINTF(("Done, %d ms\r\n", endTimeMs - startTimeMs));
#endif

#if 0
    // Scan a device for bad block by erasing, writing and reading
    uint8_t buffer[2048];

    for (int i = 0 ; i < 2048 ; i++) {
        buffer[i] = myRandom();
    }

    DPRINTF(("Complete read/write with random contents\r\n"));

    w25n01g_eraseCompletely();

    for (int page = 0 ; page < TOTAL_PAGES ; page++) {
        w25n01g_pageProgram(page * W25N01G_PAGE_SIZE, buffer, 2048);
        w25n01g_readBytes(page * W25N01G_PAGE_SIZE, buffer, 2048);
    }

    DPRINTF(("Done\r\n"));
#endif

#if 0
    // Try reading bad block marker in the extension area

    uint8_t extbuf[64];

    DPRINTF(("Reading extension area\r\n"));

    for (int page = 0 ; page < TOTAL_PAGES ; page++) {
        w25n01g_writeRegister(W25N01G_CONF_REG, W25N01G_CONFIG_BUFFER_READ_MODE|W25N01G_CONFIG_ECC_ENABLE);
        w25n01g_readExtensionBytes(page * W25N01G_PAGE_SIZE, extbuf, 64);

        uint8_t sig1, sig2;

        if (page == 0) {
            sig1 = extbuf[0];
            sig2 = extbuf[1];
            DPRINTF(("PA %x ext %x %x (REF)\r\n", page, extbuf[0], extbuf[1]));
        } else if (extbuf[0] != sig1 || extbuf[1] != sig2) {
            DPRINTF(("PA %x ext %x %x\r\n", page, extbuf[0], extbuf[1]));
        }
    }

    DPRINTF(("Done\r\n"));
#endif

#if 0
    // TEST #
    // Test PROGRAM_DATA_LOAD followed by multiple RANDOM_PROGRAM_DATA_LOAD

    DPRINTF(("Testing PROGRAM_DATA_LOAD followed by multiple RANDOM_PROGRAM_DATA_LOAD\r\n"));

    DPRINTF(("Erasing\r\n"));

    w25n01g_eraseSector(0);

    uint16_t buf[1024];

    for (int i = 0 ; i < 1024 ; i++) {
        buf[i] = i;
    }

    DPRINTF(("Loading\r\n"));

    DPRINTF(("    0\r\n"));
    w25n01g_writeEnable();
    w25n01g_programDataLoad(0, (uint8_t *)buf, 256);

    for (int chunk = 1 ; chunk < 8 ; chunk++) {
        DPRINTF(("    %d\r\n", chunk));
        w25n01g_writeEnable();
        w25n01g_randomProgramDataLoad(256 * chunk, (uint8_t *)buf + 256 * chunk, 256);
    }

    DPRINTF(("Programming\r\n"));

    w25n01g_programExecute(1); // Page address

    DPRINTF(("Reading\r\n"));

    w25n01g_readBytes(1 * W25N01G_PAGE_SIZE, (uint8_t *)buf, 2048); // Byte address

    DPRINTF(("Diff words\r\n"));

    for (int i = 0 ; i < 1024 ; i++) {
        if (buf[i] != i) {
            DPRINTF(("    %d : %d\r\n", i, buf[i]));
        }
    }

    DPRINTF(("Done\r\n"));
#endif
}

// Random stuff for testing
// http://www.ciphersbyritter.com/NEWS4/RANDC.HTM

#define znew (z=36969*(z&65535)+(z>>16))
#define wnew (w=18000*(w&65535)+(w>>16))
#define MWC ((znew<<16)+wnew )
#define SHR3 (jsr^=(jsr<<17), jsr^=(jsr>>13), jsr^=(jsr<<5))
#define CONG (jcong=69069*jcong+1234567)
#define FIB ((b=a+b),(a=b-a))
#define KISS ((MWC^CONG)+SHR3)
#define LFIB4 (c++,t[c]=t[c]+t[UC(c+58)]+t[UC(c+119)]+t[UC(c+178)])
#define SWB (c++,bro=(x<y),t[c]=(x=t[UC(c+34)])-(y=t[UC(c+19)]+bro))
#define UNI (KISS*2.328306e-10)
#define VNI ((long) KISS)*4.656613e-10
#define UC (unsigned char) /*a cast operation*/
typedef unsigned long UL;
/* Global static variables: */
static UL z=362436069, w=521288629, jsr=123456789, jcong=380116160;
static UL a=224466889, b=7584631, t[256];
/* Use random seeds to reset z,w,jsr,jcong,a,b, and the table
t[256]*/
static UL x=0,y=0,bro; static unsigned char c=0;

/* Example procedure to set the table, using KISS: */

void settable(UL i1,UL i2,UL i3,UL i4,UL i5, UL i6)
{ int i; z=i1;w=i2,jsr=i3; jcong=i4; a=i5; b=i6;
for(i=0;i<256;i=i+1) t[i]=KISS;
}

/* This is a test main program. It should compile and print 7
0's. */
void myRandomTest(void)
{
int i; UL k;
settable(12345,65435,34221,12345,9983651,95746118);
for(i=1;i<1000001;i++){k=LFIB4;} DPRINTF(("%lu\n", k-1064612766U));
for(i=1;i<1000001;i++){k=SWB ;} DPRINTF(("%lu\n", k- 627749721U));
for(i=1;i<1000001;i++){k=KISS ;} DPRINTF(("%lu\n", k-1372460312U));
for(i=1;i<1000001;i++){k=CONG ;} DPRINTF(("%lu\n", k-1529210297U));
for(i=1;i<1000001;i++){k=SHR3 ;} DPRINTF(("%lu\n", k-2642725982U));
for(i=1;i<1000001;i++){k=MWC ;} DPRINTF(("%lu\n", k- 904977562U));
for(i=1;i<1000001;i++){k=FIB ;} DPRINTF(("%lu\n", k-3519793928U));
}

void myRandomInit(void)
{
    // XXX Should call micros() or something for randomness.
    settable(12345,65435,34221,12345,9983651,95746118);
}

uint32_t myRandom(void)
{
    uint32_t k = KISS;

    return k;
}
#endif
