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

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "drivers/compass/compass.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/compass/compass_ak8963.h"

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))

static bool ak8963SpiWriteRegisterDelay(const busDevice_t *bus, uint8_t reg, uint8_t data)
{
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10);
    return true;
}

static bool ak8963SlaveReadRegisterBuffer(const busDevice_t *slavedev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address | READ_FLAG);        // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                      // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len | 0x80);              // read number of bytes
    delay(4);
    __disable_irq();
    bool ack = spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, len);    // read I2C
    __enable_irq();
    return ack;
}

static bool ak8963SlaveWriteRegister(const busDevice_t *slavedev, uint8_t reg, uint8_t data)
{
    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address); // set I2C slave address for write
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                       // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_DO, data);                       // set I2C salve value
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, 0x81);                     // write 1 byte
    return true;
}

typedef struct queuedReadState_s {
    bool waiting;
    uint8_t len;
    uint32_t readStartedAt; // time read was queued in micros.
} queuedReadState_t;

static queuedReadState_t queuedRead = { false, 0, 0};

static bool ak8963SlaveStartRead(const busDevice_t *slavedev, uint8_t reg, uint8_t len)
{
    if (queuedRead.waiting) {
        return false;
    }

    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    queuedRead.len = len;

    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_ADDR, slavedev->busdev_u.i2c.address | READ_FLAG);  // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_REG, reg);                      // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, MPU_RA_I2C_SLV0_CTRL, len | 0x80);              // read number of bytes

    queuedRead.readStartedAt = micros();
    queuedRead.waiting = true;

    return true;
}

static uint32_t ak8963SlaveQueuedReadTimeRemaining(void)
{
    if (!queuedRead.waiting) {
        return 0;
    }

    int32_t timeSinceStarted = micros() - queuedRead.readStartedAt;

    int32_t timeRemaining = 8000 - timeSinceStarted;

    if (timeRemaining < 0) {
        return 0;
    }

    return timeRemaining;
}

static bool ak8963SlaveCompleteRead(const busDevice_t *slavedev, uint8_t *buf)
{
    uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();

    const busDevice_t *bus = slavedev->busdev_u.i2c.master;

    if (timeRemaining > 0) {
        delayMicroseconds(timeRemaining);
    }

    queuedRead.waiting = false;

    spiBusReadRegisterBuffer(bus, MPU_RA_EXT_SENS_DATA_00, buf, queuedRead.len);               // read I2C buffer
    return true;
}

static bool ak8963SlaveReadData(const busDevice_t *busdev, uint8_t *buf)
{
    typedef enum {
        CHECK_STATUS = 0,
        WAITING_FOR_STATUS,
        WAITING_FOR_DATA
    } ak8963ReadState_e;

    static ak8963ReadState_e state = CHECK_STATUS;

    bool ack = false;

    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963ReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.

    bool retry = true;

restart:
    switch (state) {
        case CHECK_STATUS:
            ak8963SlaveStartRead(busdev, AK8963_MAG_REG_ST1, 1);
            state++;
            return false;

        case WAITING_FOR_STATUS: {
            uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SlaveCompleteRead(busdev, &buf[0]);

            uint8_t status = buf[0];

            if (!ack || (status & ST1_DATA_READY) == 0) {
                // too early. queue the status read again
                state = CHECK_STATUS;
                if (retry) {
                    retry = false;
                    goto restart;
                }
                return false;
            }


            // read the 6 bytes of data and the status2 register
            ak8963SlaveStartRead(busdev, AK8963_MAG_REG_HXL, 7);

            state++;

            return false;
        }

        case WAITING_FOR_DATA: {
            uint32_t timeRemaining = ak8963SlaveQueuedReadTimeRemaining();
            if (timeRemaining) {
                return false;
            }

            ack = ak8963SlaveCompleteRead(busdev, &buf[0]);
            state = CHECK_STATUS;
        }
    }

    return ack;
}
#endif

static bool ak8963ReadRegisterBuffer(const busDevice_t *busdev, uint8_t reg, uint8_t len, uint8_t *buf)
{
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        return i2cBusReadRegisterBuffer(busdev, reg, buf, len);
#endif

#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        return spiBusReadRegisterBuffer(busdev, reg, buf, len);
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_SLAVE:
        return ak8963SlaveReadRegisterBuffer(busdev, reg, buf, len);
#endif
    }
    return false;
}

static bool ak8963WriteRegister(const busDevice_t *busdev, uint8_t reg, uint8_t data)
{
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        return i2cBusWriteRegister(busdev, reg, data);
#endif

#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        return spiBusWriteRegister(busdev, reg, data);
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_SLAVE:
        return ak8963SlaveWriteRegister(busdev, reg, data);
#endif
    }
    return false;
}

static bool ak8963ReadData(const busDevice_t *busdev, uint8_t *buf)
{
    uint8_t status;

    bool ack = ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, 1, &status);

    if (!ack || (status & ST1_DATA_READY) == 0) {
        return false;
    }

    return ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_HXL, 7, buf);
}

static bool ak8963Read(magDev_t *magdev, int16_t *magData)
{
    bool ack = false;
    uint8_t buf[7];

    const busDevice_t *busdev = &magdev->busdev;

    switch (busdev->bustype) {
#if defined(USE_MAG_SPI_AK8963) || defined(USE_MAG_AK8963)
    case BUSTYPE_I2C:
    case BUSTYPE_SPI:
        ack = ak8963ReadData(busdev, buf);
        break;
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_SLAVE:
        ack = ak8963SlaveReadData(busdev, buf);
        break;
#endif
    }

    uint8_t status2 = buf[6];
    if (!ack || (status2 & ST2_DATA_ERROR) || (status2 & ST2_MAG_SENSOR_OVERFLOW)) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * magGain[Y];
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * magGain[Z];

    return ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE); // start reading again
}

static bool ak8963Init(magDev_t *magdev)
{
    uint8_t calibration[3];
    uint8_t status;

    const busDevice_t *busdev = &magdev->busdev;

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down before entering fuse mode
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_FUSE_ROM); // Enter Fuse ROM access mode
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ASAX, sizeof(calibration), calibration); // Read the x-, y-, and z-axis calibration values

    magGain[X] = ((((float)(int8_t)calibration[X] - 128) / 256) + 1) * 30;
    magGain[Y] = ((((float)(int8_t)calibration[Y] - 128) / 256) + 1) * 30;
    magGain[Z] = ((((float)(int8_t)calibration[Z] - 128) / 256) + 1) * 30;

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_POWER_DOWN); // power down after reading.

    // Clear status registers
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST1, 1, &status);
    ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_ST2, 1, &status);

    // Trigger first measurement
    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL1, CNTL1_MODE_ONCE);
    return true;
}

void ak8963BusInit(const busDevice_t *busdev)
{
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        UNUSED(busdev);
        break;
#endif

#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_COMPASS_CS, 0);
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_OUT_PP);
        IOHi(busdev->busdev_u.spi.csnPin); // Disable
        spiSetDivisor(busdev->busdev_u.spi.instance, SPI_CLOCK_STANDARD); // XXX
        break;
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_SLAVE: 
        // initialze I2C master via SPI bus
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.i2c.master, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR | MPU6500_BIT_BYPASS_EN);
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.i2c.master, MPU_RA_I2C_MST_CTRL, 0x0D);                // I2C multi-master / 400kHz
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.i2c.master, MPU_RA_USER_CTRL, 0x30);                   // I2C master mode, SPI mode only
        break;
#endif
    }
}

void ak8963BusDeInit(const busDevice_t *busdev)
{
    switch (busdev->bustype) {
#ifdef USE_MAG_AK8963
    case BUSTYPE_I2C:
        UNUSED(busdev);
        break;
#endif

#ifdef USE_MAG_SPI_AK8963
    case BUSTYPE_SPI:
        IOConfigGPIO(busdev->busdev_u.spi.csnPin, IOCFG_IPU);
        IORelease(busdev->busdev_u.spi.csnPin);
        IOInit(busdev->busdev_u.spi.csnPin, OWNER_SPI_PREINIT, 0);
        break;
#endif

#if defined(USE_MAG_AK8963) && (defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250))
    case BUSTYPE_SLAVE:
        ak8963SpiWriteRegisterDelay(busdev->busdev_u.i2c.master, MPU_RA_INT_PIN_CFG, MPU6500_BIT_INT_ANYRD_2CLEAR);
        break;
#endif
    }
}

bool ak8963Detect(magDev_t *mag)
{
    uint8_t sig = 0;

    busDevice_t *busdev = &mag->busdev;

    ak8963BusInit(busdev);

    ak8963WriteRegister(busdev, AK8963_MAG_REG_CNTL2, CNTL2_SOFT_RESET); // reset MAG
    delay(4);

    bool ack = ak8963ReadRegisterBuffer(busdev, AK8963_MAG_REG_WIA, 1, &sig);  // check for AK8963

    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        mag->init = ak8963Init;
        mag->read = ak8963Read;

        return true;
    }

    ak8963BusDeInit(busdev);

    return false;
}
