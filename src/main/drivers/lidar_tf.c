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

#ifdef USE_LIDAR_TF

#include "build/debug.h"
#include "build/build_config.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/serial.h"
#include "drivers/altimeter.h"

#include "lidar_tf.h"

#define TF_FRAME_LENGTH    6             // Exclusing sync bytes (0x59) x 2 and checksum
#define TF_FRAME_SYNC_BYTE 0x59
#define TF_TIMEOUT_US      (100 * 1000)  // Maximum time to wait for valid frame

//
// Benewake TFmini frame format
// Byte
// -: SYNC
// -: SYNC
// 0: Measured distance (LSB)
// 1: Measured distance (MSB)
// 2: Signal strength (LSB)
// 3: Signal strength (MSB)
// 4: Integral time
// 5: Reserved
// -: Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Note *1: TFmini product specification (Version A00) specifies byte 6 is
// reserved and byte 7 is quality, but it seems like byte 7 contains
// value of 7 for good measurement and 2 for bad measurement.
//
// Credibility
// 1. If distance is 12m (1200cm), then OoR.
//
#define TF_MINI_FRAME_INTEGRAL_TIME 4
#define TF_MINI_RANGE_MIN 40
#define TF_MINI_RANGE_MAX 1200
#define TF_DETECTION_CONE_DECIDEGREES 450 // Very conservative.

//
// Benewake TF02 frame format (From SJ-GU-TF02-01 Version: A01)
// Byte
// -: SYNC
// -: SYNC
// 0: Measured distance (LSB)
// 1: Measured distance (MSB)
// 2: Signal strength (LSB)
// 3: Signal strength (MSB)
// 4: SIG (Reliability in 1~8, less than 7 is unreliable)
// 5: TIME (Exposure time, 3 or 6)
// -: Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If SIG is less than 7, unreliable
// 2. If distance is 22m (2200cm), then OoR.
//
#define TF_02_FRAME_SIG 4
#define TF_02_RANGE_MIN 40
#define TF_02_RANGE_MAX 2200
#define TF_02_DETECTION_CONE_DECIDEGREES 45

static altimeterRange_t lidarTFRange;
static altimeterDevice_t lidarTFDevice; // Forward

static serialPort_t *tfSerialPort = NULL;

typedef enum {
    TF_FRAME_STATE_WAIT_START1,
    TF_FRAME_STATE_WAIT_START2,
    TF_FRAME_STATE_READING_PAYLOAD,
    TF_FRAME_STATE_WAIT_CKSUM,
} tfFrameState_e;

static tfFrameState_e tfFrameState;

PG_REGISTER_WITH_RESET_TEMPLATE(lidarTFConfig_t, lidarTFConfig, PG_LIDAR_TF_CONFIG, 0);

PG_RESET_TEMPLATE(lidarTFConfig_t, lidarTFConfig,
    .device = LIDAR_TF_TYPE_TFMINI,
);

static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfReceivePosition;

// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static uint8_t tfCmdTFmini[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

// TF02
// Same as TFmini for now..
static uint8_t tfCmdTF02[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

static int32_t lidarTFValue;
static uint16_t lidarTFerrors = 0;

static void lidarTFSendCommand(void)
{
    switch (lidarTFConfig()->device) {
    case LIDAR_TF_TYPE_TFMINI:
        serialWriteBuf(tfSerialPort, tfCmdTFmini, ARRAYLEN(tfCmdTFmini));
        break;
    case LIDAR_TF_TYPE_TF02:
        serialWriteBuf(tfSerialPort, tfCmdTF02, ARRAYLEN(tfCmdTF02));
        break;
    }
}

altimeterDevice_t *lidarTFInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);

    if (!portConfig) {
        return NULL;
    }

    tfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, MODE_RXTX, 0);

    if (tfSerialPort == NULL) {
        return NULL;
    }

    lidarTFRange.maxRangeCm = (lidarTFConfig()->device == LIDAR_TF_TYPE_TFMINI) ? TF_MINI_RANGE_MAX : TF_02_RANGE_MAX;
    lidarTFRange.detectionConeDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;
    lidarTFRange.detectionConeExtendedDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;

    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfReceivePosition = 0;

    return &lidarTFDevice;
}

void lidarTFUpdate(timeUs_t currentTimeUs)
{
    static timeUs_t lastFrameReceivedUs = 0;

    if (tfSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(tfSerialPort)) {
        uint8_t c = serialRead(tfSerialPort);
        switch (tfFrameState) {
        case TF_FRAME_STATE_WAIT_START1:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfFrameState = TF_FRAME_STATE_WAIT_START2;
            }
            break;

        case TF_FRAME_STATE_WAIT_START2:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfFrameState = TF_FRAME_STATE_READING_PAYLOAD;
            } else {
                tfFrameState = TF_FRAME_STATE_WAIT_START1;
            }
            break;

        case TF_FRAME_STATE_READING_PAYLOAD:
            tfFrame[tfReceivePosition++] = c;
            if (tfReceivePosition == TF_FRAME_LENGTH) {
                tfFrameState = TF_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case TF_FRAME_STATE_WAIT_CKSUM:
            {
                uint8_t cksum = TF_FRAME_SYNC_BYTE + TF_FRAME_SYNC_BYTE;
                for (int i = 0 ; i < TF_FRAME_LENGTH ; i++) {
                    cksum += tfFrame[i];
                }

                if (c == cksum) {

                    uint16_t distance = tfFrame[0] | (tfFrame[1] << 8);
                    uint16_t strength = tfFrame[2] | (tfFrame[3] << 8);

                    DEBUG_SET(DEBUG_LIDAR_TF, 0, distance);
                    DEBUG_SET(DEBUG_LIDAR_TF, 1, strength);
                    DEBUG_SET(DEBUG_LIDAR_TF, 2, tfFrame[4]);
                    DEBUG_SET(DEBUG_LIDAR_TF, 3, tfFrame[5]);

                    switch (lidarTFConfig()->device) {
                    case LIDAR_TF_TYPE_TFMINI:
                        if (distance >= TF_MINI_RANGE_MIN && distance < TF_MINI_RANGE_MAX) {
                            lidarTFValue = distance;
                            if (tfFrame[TF_MINI_FRAME_INTEGRAL_TIME] == 7) {
                                // When integral time is long (7), measured distance tends to be longer by 12~13.
                                lidarTFValue -= 13;
                            }
                        } else {
                            lidarTFValue = -1;
                        }
                        break;

                    case LIDAR_TF_TYPE_TF02:
                        if (distance >= TF_02_RANGE_MIN && distance < TF_02_RANGE_MAX && tfFrame[TF_02_FRAME_SIG] >= 7) {
                            lidarTFValue = distance;
                        } else {
                            lidarTFValue = -1;
                        }
                        break;
                    }
                    lastFrameReceivedUs = currentTimeUs;
                } else {
                    // Checksum error. Simply discard the current frame.
                    ++lidarTFerrors;
                    //DEBUG_SET(DEBUG_LIDAR_TF, 3, lidarTFerrors);
                }
            }

            tfFrameState = TF_FRAME_STATE_WAIT_START1;
            tfReceivePosition = 0;

            break;
        }
    }

    // If valid frame hasn't been received for more than a timeout, resend command.

    if (currentTimeUs - lastFrameReceivedUs > TF_TIMEOUT_US) {
        lidarTFSendCommand();
    }
}

// Return device output in cm

int32_t lidarTFGetDistance(void)
{
    return lidarTFValue;
}

static altimeterVTable_t lidarTFVTable = {
    .startReading = lidarTFUpdate,
    .getDistance = lidarTFGetDistance,
};

static altimeterDevice_t lidarTFDevice = {
    .range = &lidarTFRange,
    .vTable = &lidarTFVTable
};

#endif
