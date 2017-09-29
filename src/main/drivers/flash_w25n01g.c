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
#define W25N01G_INSTRUCTION_RANDOM_LOAD_PROGRAM_DATA 0x84
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
#define W25N01G_BBLUT_LBA_ENABLED (1 << 15)
#define W25N01G_BBLUT_LBA_INVALID (1 << 14)

// Some useful defs and macros
#define W25N01G_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_PAGE(laddr) ((laddr) / W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_BLOCK(laddr) (W25N01G_LINEAR_TO_PAGE(laddr) / W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_PAGE(block) ((block) * W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_LINEAR(block) (W25N01G_BLOCK_TO_PAGE(block) * W25N01G_PAGE_SIZE)

// BB replacement area
#define W25N01G_BBAREA_BLOCKS 32
#define W25N01G_BBAREA_START_BLOCK (W25N01G_BLOCKS_PER_DIE - W25N01G_BBAREA_BLOCKS)

// XXX These will be gone
#define DISABLE_M25P16       IOHi(bus->busdev_u.spi.csnPin); __NOP()
#define ENABLE_M25P16        __NOP(); IOLo(bus->busdev_u.spi.csnPin)

static busDevice_t *bus;

// The timeout values (2ms minimum to avoid 1 tick advance in consecutive calls to millis).
#define W25N01G_TIMEOUT_PAGE_READ_MS      2   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_MS   2   // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS   15   // tBEmax = 10ms

typedef struct bblut_s {
    uint16_t pba;
    uint16_t lba;
} bblut_t;

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

bool w25n01g_isReady(void)
{
#if 0
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    couldBeBusy = couldBeBusy && ((w25n01g_readRegister(W25N01G_STAT_REG) & W25N01G_STATUS_FLAG_BUSY) != 0);

    return !couldBeBusy;
#else
    return ((w25n01g_readRegister(W25N01G_STAT_REG) & W25N01G_STATUS_FLAG_BUSY) == 0);
#endif
}

bool w25n01g_waitForReady(uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!w25n01g_isReady()) {
        if (millis() - time > timeoutMillis) {
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

    geometry.sectors -= W25N01G_BBAREA_BLOCKS;

    geometry.sectorSize = geometry.pagesPerSector * geometry.pageSize;
    geometry.totalSize = geometry.sectorSize * geometry.sectors;

    couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

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

    // Protection to upper 1/32 (BP[3:0] = 0101, TB=0), WP-E on
    //w25n01g_writeRegister(W25N01G_PROT_REG, (5 << 3)|(0 << 2)|(1 << 1));

    // No protection, WP-E on
    w25n01g_writeRegister(W25N01G_PROT_REG, (0 << 3)|(0 << 2)|(1 << 1));

    // Continuous mode (BUF = 0), ECC enabled (ECC = 1)
    w25n01g_writeRegister(W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE);

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

//
// Writes are done in three steps:
// (1) Load internal data buffer with data to write
//     - We use "Random Load Program Data", as "Load Program Data" resets unused data bytes in the buffer to 0xff.
//     - Each "Random Load Program Data" instruction must be accompanied by at least a single data.
//     - Each "Random Load Program Data" instruction terminates at the rising of CS.
// (2) Enable write
// (3) Issue "Execute Program"
//

static uint32_t programStartAddress;
static uint32_t programLoadAddress;

void w25n01g_pageProgramBegin(uint32_t address)
{
    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    w25n01g_writeEnable();

    programStartAddress = programLoadAddress = address;
}

void w25n01g_pageProgramContinue(const uint8_t *data, int length)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_RANDOM_LOAD_PROGRAM_DATA, W25N01G_LINEAR_TO_COLUMN(programLoadAddress) >> 8, W25N01G_LINEAR_TO_COLUMN(programLoadAddress) & 0xff };

    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    spiTransfer(bus->busdev_u.spi.instance, data, NULL, length);
    DISABLE_M25P16;

    programLoadAddress += length;
}

void w25n01g_pageProgramFinish(void)
{
    const uint8_t cmd[] = { W25N01G_INSTRUCTION_PROGRAM_EXECUTE, 0, W25N01G_LINEAR_TO_PAGE(programStartAddress) >> 8, W25N01G_LINEAR_TO_PAGE(programStartAddress) & 0xff };

    w25n01g_waitForReady(W25N01G_TIMEOUT_PAGE_PROGRAM_MS);

    ENABLE_M25P16;
    spiTransfer(bus->busdev_u.spi.instance, cmd, NULL, sizeof(cmd));
    DISABLE_M25P16;
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

void w25n01g_addError(uint32_t address, uint8_t code)
{
    DPRINTF(("addError: PA %x code %d\r\n", W25N01G_LINEAR_TO_PAGE(address), code));
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

static uint32_t currentReadAddress = UINT32_MAX;

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

void myRandomInit(void);
uint32_t myRandom(void);

void w25n01g_deviceInit(void)
{
    int olines = 0;

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

#if 1
    // Test #4
    // Write few pages with their page numbers.
    // Read entire device and see if non-empty page has its page number written.

#define TOTAL_PAGES (W25N01G_PAGES_PER_BLOCK * W25N01G_BLOCKS_PER_DIE)
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
}

// Random stuff

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
}

uint32_t myRandom(void)
{
    uint32_t k = KISS;

    return k;
}
#endif
