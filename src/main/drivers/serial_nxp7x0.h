
#pragma once

#define UUXSERIAL_BUFFER_SIZE 256

typedef enum {
    UUXSERIAL1 = 0,
    UUXSERIAL2,
    UUXSERIAL3,
    UUXSERIAL4,
    UUXSERIAL5,
    UUXSERIAL6,
    UUXSERIAL7,
    UUXSERIAL8,
} uuxSerialPortIndex_e;

serialPort_t *openUUXSerial(
	uuxSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options);

// serialPort API
void uuxSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t uuxSerialRxBytesWaiting(serialPort_t *instance);
uint8_t uuxSerialTxBytesFree(serialPort_t *instance);
uint8_t uuxSerialReadByte(serialPort_t *instance);
void uuxSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate);
bool uuxSerialTransmitBufferEmpty(serialPort_t *instance);

// i2c poller
void uuxSerialPoller(void);
