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

#pragma once

#include <stdint.h>
#include "drivers/io_types.h"

typedef struct flashGeometry_s {
    uint8_t dieCount;
    uint16_t sectors; // Count of the number of erasable blocks on the device
    const uint16_t pageSize; // In bytes
    uint32_t sectorSize; // This is just pagesPerSector * pageSize
    uint32_t totalSize;  // This is just sectorSize * sectors
    uint16_t pagesPerSector;
} flashGeometry_t;

typedef struct flashConfig_s {
    ioTag_t csTag;
    uint8_t spiDevice;
} flashConfig_t;

bool flashInit(const flashConfig_t *flashConfig);

bool flashIsReady(void);
bool flashWaitForReady(uint32_t timeoutMillis);
void flashEraseSector(uint32_t address);
void flashEraseCompletely(void);
void flashPageProgramBegin(uint32_t address);
void flashPageProgramContinue(const uint8_t *data, int length);
void flashPageProgramFinish(void);
void flashPageProgram(uint32_t address, const uint8_t *data, int length);
int flashReadBytes(uint32_t address, uint8_t *buffer, int length);
const flashGeometry_t * flashGetGeometry(void);

// VTable should go to impl.h
typedef struct flashVTable_s {
    bool (*isReady)(void);
    bool (*waitForReady)(uint32_t timeoutMillis);
    void (*eraseSector)(uint32_t address);
    void (*eraseCompletely)(void);
    void (*pageProgramBegin)(uint32_t address);
    void (*pageProgramContinue)(const uint8_t *data, int length);
    void (*pageProgramFinish)(void);
    void (*pageProgram)(uint32_t address, const uint8_t *data, int length);
    int (*readBytes)(uint32_t address, uint8_t *buffer, int length);
    const flashGeometry_t *(*getGeometry)(void);
} flashVTable_t;

