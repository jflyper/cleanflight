#pragma once

#include "drivers/vtx_common.h" // for vtxConfig_t

// Downward API
typedef struct gen6705Device_s {
    void (*writeRegister)(uint8_t reg, uint32_t data);
} gen6705Device_t;

void gen6705RegisterDevice(gen6705Device_t *gen6705Device);

void gen6705Init(void);

void gen6705SetBandChan(uint8_t band, uint8_t chan);
bool gen6705GetBandChan(uint8_t *pBand, uint8_t *pChan);
