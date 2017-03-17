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

// Interface to cli variables

#pragma once

typedef struct vtxConfig_s {
    uint8_t vtx_device; // External=0, Internal SPI=1, Internal SoftSPI=2
    uint8_t vtx_mode;   // 0=band/chan, 1=direct freq, 2=vtxrc
    uint8_t vtx_band;
    uint8_t vtx_channel;
    uint8_t vtx_power;
    uint8_t vtx_opmodel;
    uint16_t vtx_mhz;
} vtxConfig_t;

vtxConfig_t vtxConfig;

// Device type for vtx_device CLI var.
// Initialized based on per target RTC6705 driver usage, can be modified by CLI.
typedef enum {
    VTX_DEVICE_OTHER = 0,
    VTX_DEVICE_RTC6705_SPI,
    VTX_DEVICE_RTC6705_SOFTSPI,
} vtxDevice_e;

// For MSP
typedef enum {
    VTXDEV_UNSUPPORTED = 0, // reserved for MSP
    VTXDEV_GEN6705 = 1,
    // 2 reserved
    VTXDEV_SMARTAUDIO = 3,
    VTXDEV_TRAMP      = 4,
    VTXDEV_UNKNOWN    = 0xFF,
} vtxDevType_e;

typedef struct vtxDeviceParam_s {
    uint8_t numBand;
    uint8_t numChan;
    uint8_t numPower;
    uint16_t freqMin;
    uint16_t freqMax;
    uint16_t *freqTable;  // Array of [numBand][numChan]
    char **bandNames;    // char *bandNames[numBand]
    char **chanNames;    // char *chanNames[numChan]
    char **powerNames;   // char *powerNames[numPower]
    char const *bandLetters;   // char bandLetters[numPower]
} vtxDeviceParam_t;

struct vtxVTable_s;

typedef struct vtxDevice_s {
    const struct vtxVTable_s *vTable;
    vtxDeviceParam_t devParam; // Mutable so SmartAudio driver can rewrite some fields.
} vtxDevice_t;

// {set,get}BandChan: band and chan are 1 origin
// {set,get}PowerByIndex: 0 = Power OFF, 1 = device dependent
// {set,get}Pitmode: 0 = OFF, 1 = ON

typedef struct vtxVTable_s {
    void (*process)(uint32_t currentTimeUs);
    vtxDevType_e (*getDeviceType)(void);
    bool (*isReady)(void);

    void (*setFselMode)(uint8_t mode); // Band/channel(0) or Direct frequency(1)
    void (*setBandChan)(uint8_t band, uint8_t chan);
    void (*setFreq)(uint16_t freq);
    void (*setPowerByIndex)(uint8_t level);
    void (*setPitmode)(uint8_t onoff);

    bool (*getFselMode)(uint8_t *pMode); // Band/channel(0) or Direct frequency(1)
    bool (*getBandChan)(uint8_t *pBand, uint8_t *pChan);
    bool (*getFreq)(uint16_t *pFreq);
    bool (*getPowerIndex)(uint8_t *pIndex);
    bool (*getPitmode)(uint8_t *pOnoff);
} vtxVTable_t;

// 3.1.0
// PIT mode is defined as LOWEST POSSIBLE RF POWER.
// - It can be a dedicated mode, or lowest RF power possible.
// - It is *NOT* RF on/off control ?

void vtxCommonInit(vtxConfig_t *pVtxConfigToUse);
void vtxCommonRegisterDevice(vtxDevice_t *pDevice);

// VTable functions
void vtxCommonProcess(uint32_t currentTimeUs);
uint8_t vtxCommonGetDeviceType(void);

void vtxCommonSetBandChan(uint8_t band, uint8_t chan);
void vtxCommonSetFreq(uint16_t freq);
void vtxCommonSetPowerByIndex(uint8_t level);
void vtxCommonSetFselMode(uint8_t mode);
void vtxCommonSetPitmode(uint8_t onoff);

bool vtxCommonGetBandChan(uint8_t *pBand, uint8_t *pChan);
bool vtxCommonGetFreq(uint16_t *pFreq);
bool vtxCommonGetPowerIndex(uint8_t *pIndex);
bool vtxCommonGetFselMode(uint8_t *pMode);
bool vtxCommonGetPitmode(uint8_t *pOnoff);

// V1.1 API
vtxDeviceParam_t *vtxCommonGetDeviceParam(void);
