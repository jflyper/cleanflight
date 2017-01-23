#pragma once

typedef struct gen6705Device_s {
    void (*writeRegister)(uint8_t reg, uint32_t data);
} gen6705Device_t;

void gen6705Init(void);
void gen6705RegisterDevice(gen6705Device_t *gen6705Device);
