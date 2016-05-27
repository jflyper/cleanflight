#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/utils.h"
#include "common/atomic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "nvic.h"
//#include "timer.h"

#include "target.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/system.h"

#include "serial.h"
#include "serial_nxp7x0.h"

#include "sc16is7x0.h"    // NXP SC16IS7xx registers
#include "ubreg.h"        // I2C/UART Bridge specific registers

#if 1
#include "common/printf.h"
#define dprintf(x) printf x
#else
#define dprintf(x)
#endif

//
// Watch out!!!
// Slaves with ATmega328 running at 16MHz can't handle back-to-back WRITEs.
// @800KHz, 45usec delay is required.
// 
#define UBdelayMicroseconds(n) delayMicroseconds(n)

#ifdef NAZE
#define UUXSERIAL_SPI_INSTANCE NAZE_SPI_INSTANCE
#define UUXSERIAL_SPI_CS_GPIO  NAZE_SPI_CS_GPIO
#define UUXSERIAL_SPI_CS_PIN   NAZE_SPI_CS_PIN
#endif

#ifdef SPRACINGF3
#define UUXSERIAL_SPI_INSTANCE SPI2
#define UUXSERIAL_SPI_CS_GPIO  GPIOB
#define UUXSERIAL_SPI_CS_PIN   GPIO_Pin_4 // RC5(YELLOW)
#endif

#define DISABLE_NXP(uux)  GPIO_SetBits((uux)->spicsgpio, (uux)->spicspin)
#define ENABLE_NXP(uux)   {\
    spiSetSettings(uux->spibus, &uux->spisettings);\
    GPIO_ResetBits((uux)->spicsgpio, (uux)->spicspin);}


// XXX Should make this variable based on bus clock & device speed ...
//#define UUXSERIAL_I2C_MAX_RXFRAG 16
//#define UUXSERIAL_I2C_MAX_TXFRAG 16 
#define UUXSERIAL_I2C_MAX_RXFRAG 8
#define UUXSERIAL_I2C_MAX_TXFRAG 8 

// For SC16IS{74x,75x} on 4MHz (250ns/0.25us) SPI bus:
// 64 bytes = 250 * 8 * 64 = 128us

#define UUXSERIAL_SPI_MAX_RXFRAG 64
#define UUXSERIAL_SPI_MAX_TXFRAG 64

// 16B at 100Hz service interval = 16*100 = 1.6KB/sec = 16kbps
// On 800KHz bus, 1KHz slot is 800 bits = 88 bytes
// 0x68 read = addr68+reg43+addr+6datar = 9 bytes
// 0x68 read = addr68+reg3b+addr+6datar = 9 bytes
 
// Per device function vector (not used yet)

struct uuxSerial_s; // Forward declaration

typedef struct devops_s {
    bool (*reset)(struct uuxSerial_s *);
    void (*init)(struct uuxSerial_s *);

    void (*setSpeed)(struct uuxSerial_s *);
    void (*setLine)(struct uuxSerial_s *);
    void (*transmitterControl)(struct uuxSerial_s *, bool);
    void (*receiverControl)(struct uuxSerial_s *, bool);

    void (*getRxLvl)(struct uuxSerial_s *);
    void (*getTxLvl)(struct uuxSerial_s *);
    void (*receive)(struct uuxSerial_s *, int count);
    void (*transmit)(struct uuxSerial_s *, int count);
} devops_t;

// XXX device hierachy?
// XXX E.g. SC16IS7{5,6}2
// mcu
//   bus(SPIx)
//     controller(SC16IS7{5,6}2)
//       device (uart chan 0)
//       device (uart chan 1)
//       device (gpio chan 0)
//       device (gpio chan 1)
//       device (optional --unimplemented-- interrupt encoder)

typedef struct uuxSerial_s {
    serialPort_t     port;    // Must be the first
    uint8_t          rxBuf[UUXSERIAL_BUFFER_SIZE];
    uint8_t          txBuf[UUXSERIAL_BUFFER_SIZE];
    uint8_t          uuxSerialPortIndex;

    bool             active;
    uint8_t          bustype;
#define UUXSERIAL_BUSTYPE_I2C 0
#define UUXSERIAL_BUSTYPE_SPI 1
    // I2c
    I2C_TypeDef      *i2cbus;
    uint8_t          addr;
    // SPI
    SPI_TypeDef      *spibus;
    GPIO_TypeDef     *spicsgpio;
    uint16_t         spicspin;
    spiSettings_t    spisettings;

    uint8_t          chan;               // SC16IS7{5,6}2 has two chans

    uint8_t          devtype;
#define UUXSERIAL_DEVTYPE_NONE         0
#define UUXSERIAL_DEVTYPE_NXP          1 // NXP SC16IS74{0,1},7{5,6}{0,2}
#define UUXSERIAL_DEVTYPE_MINIMAL      2 // UART Bridge
#define UUXSERIAL_DEVTYPE_UBLOX        3 // u-blox DDC

#define UBX_REG_LENHI    0xfd
#define UBX_REG_LENLO    0xfe
#define UBX_REG_DATA     0xff

    devops_t         devops;   // XXX not used yet

    uint16_t         apiver;

    int32_t         freq;      // -1:don't care, 0:baudrate/150, otherwise::crystal
#define UB_FREQ_DONTCARE   -1
#define UB_FREQ_BAUDx150    0

    int8_t           polled;

    //uint8_t          cycletime; // Average time per byte

    uint8_t          rxlvl;
    uint8_t          txlvl;

    uint8_t          iir;       // Last IIR read
    uint8_t          fcr;       // FCR soft copy
    uint8_t          efcr;      // EFCR soft copy

    int              bcycle;

} uuxSerial_t;

extern uuxSerial_t uuxSerialPorts[];
extern const struct serialPortVTable uuxSerialVTable[];

#ifdef USE_UUXSERIAL1
#define MAX_UUXSERIAL_PORTS 1
#endif
#ifdef USE_UUXSERIAL2
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 2
#endif
#ifdef USE_UUXSERIAL3
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 3
#endif
#ifdef USE_UUXSERIAL4
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 4
#endif
#ifdef USE_UUXSERIAL5
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 5
#endif
#ifdef USE_UUXSERIAL6
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 6
#endif
#ifdef USE_UUXSERIAL7
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 7
#endif
#ifdef USE_UUXSERIAL8
#undef MAX_UUXSERIAL_PORTS
#define MAX_UUXSERIAL_PORTS 8
#endif

static bool uuxSerialPortsInit = false;
uuxSerial_t uuxSerialPorts[MAX_UUXSERIAL_PORTS];

/*
 * Buffer management
 */
#define rxBufferLen(port) ((((port).rxBufferHead - (port).rxBufferTail)) & ((port).rxBufferSize - 1))
#define txBufferLen(port) ((((port).txBufferHead - (port).txBufferTail)) & ((port).txBufferSize - 1))

#define rxBufferRoom(port) ((port).rxBufferSize - rxBufferLen(port) - 1)
#define txBufferRoom(port) ((port).txBufferSize - txBufferLen(port) - 1)

#define rxBufferBurstLimit(port) ((port).rxBufferSize - (port).rxBufferHead)
#define txBufferBurstLimit(port) ((port).txBufferSize - (port).txBufferTail)

static void resetBuffers(uuxSerial_t *uux)
{
    uux->port.rxBufferSize = UUXSERIAL_BUFFER_SIZE;
    uux->port.rxBuffer = uux->rxBuf;
    uux->port.rxBufferTail = 0;
    uux->port.rxBufferHead = 0;

    uux->port.txBufferSize = UUXSERIAL_BUFFER_SIZE;
    uux->port.txBuffer = uux->txBuf;
    uux->port.txBufferTail = 0;
    uux->port.txBufferHead = 0;
}

/*
 * Interrupt related
 */
volatile bool nxpInterrupted = false;

// Some experimental pin assignments

#ifdef SPRACINGF3
// RC2 (BLUE) = PA1 : IRQ
// (GREEN) = PB5
// (YELLOW) = PB4
// SPRF3
#if 1
// RC2 (BLUE) = PA1 : Conflict with PB1 (RC8 Sonar Echo)
#define UUXSERIAL_INT_PERIPH          RCC_AHBPeriph_GPIOA
#define UUXSERIAL_INT_PIN             Pin_1
#define UUXSERIAL_INT_GPIO            GPIOA
#define UUXSERIAL_INT_EXTI_LINE       EXTI_Line1
#define UUXSERIAL_INT_EXTI_PIN_SOURCE EXTI_PinSource1
#define UUXSERIAL_INT_IRQN            EXTI1_IRQn
#endif

#if 0
// RC5 (YELLOW) = PB4 : PA4 = VBAT ADC (conflict? check ADC config!)
#define UUXSERIAL_INT_PERIPH          RCC_AHBPeriph_GPIOB
#define UUXSERIAL_INT_PIN             Pin_4
#define UUXSERIAL_INT_GPIO            GPIOB
#define UUXSERIAL_INT_EXTI_LINE       EXTI_Line4
#define UUXSERIAL_INT_EXTI_PIN_SOURCE EXTI_PinSource4
#define UUXSERIAL_INT_IRQN            EXTI4_IRQn
#endif

// RC6 (GREEN) = PB5 : PA5 = CURRENT ADC (conflict? check ADC config!)
#endif

#ifdef NAZE
#define UUXSERIAL_INT_PERIPH          RCC_APB2Periph_GPIOA
#define UUXSERIAL_INT_PIN             Pin_1
#define UUXSERIAL_INT_GPIO            GPIOA
#define UUXSERIAL_INT_EXTI_LINE       EXTI_Line1
#define UUXSERIAL_INT_EXTI_PIN_SOURCE GPIO_PinSource1
#define UUXSERIAL_INT_IRQN            EXTI1_IRQn
#endif

extiConfig_t uuxIntExtiConfig = {
#ifdef STM32F10X
    .gpioAPB2Peripherals = RCC_APB2Periph_GPIOB,
#endif
#ifdef STM32F303xC
    .gpioAHBPeripherals	 = RCC_AHBPeriph_GPIOB,
#endif

    .gpioPort            = UUXSERIAL_INT_GPIO,
    .gpioPin             = UUXSERIAL_INT_PIN,

    .exti_port_source    = UUXSERIAL_INT_GPIO,
    .exti_pin_source     = UUXSERIAL_INT_EXTI_PIN_SOURCE,
    .exti_line           = UUXSERIAL_INT_EXTI_LINE,
    .exti_irqn           = UUXSERIAL_INT_IRQN,
};

void uuxSerial_EXTI_Handler(void)
{
    if (EXTI_GetITStatus(uuxIntExtiConfig.exti_line) == RESET) {
        return;
    }

    // digitalHi(GPIOB, Pin_5); // Debugging

    EXTI_ClearITPendingBit(uuxIntExtiConfig.exti_line);

    nxpInterrupted = true;

    // digitalLo(GPIOB, Pin_5); // Debugging
}

void uuxSerialConfigureEXTI(void)
{
#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(uuxIntExtiConfig.exti_port_source, uuxIntExtiConfig.exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(uuxIntExtiConfig.exti_port_source, uuxIntExtiConfig.exti_pin_source);
#endif

    registerExtiCallbackHandler(uuxIntExtiConfig.exti_irqn, uuxSerial_EXTI_Handler);

    EXTI_ClearITPendingBit(uuxIntExtiConfig.exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = uuxIntExtiConfig.exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = uuxIntExtiConfig.exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_SERIAL);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_SERIAL);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void uuxSerialIntExtiInit(void)
{
    gpio_config_t gpio;

    static bool uuxExtiInitDone = false;

    //if (uuxExtiInitDone || !uuxIntExtiConfig) {
    if (uuxExtiInitDone) {
        return;
    }

#ifdef STM32F10X
        if (uuxIntExtiConfig.gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(uuxIntExtiConfig.gpioAPB2Peripherals, ENABLE);
        }
#endif
#ifdef STM32F303
        if (uuxIntExtiConfig.gpioAHBPeripherals) {
            RCC_AHBPeriphClockCmd(uuxIntExtiConfig.gpioAHBPeripherals, ENABLE);
        }
#endif

    gpio.pin = uuxIntExtiConfig.gpioPin;
    gpio.speed = Speed_2MHz;
    //gpio.mode = Mode_IN_FLOATING;
    //gpio.mode = Mode_Out_PP; // For port connectivity testing
    gpio.mode = Mode_IPU; // Input with pull-up
    gpioInit(uuxIntExtiConfig.gpioPort, &gpio);

    uuxSerialConfigureEXTI();

    uuxExtiInitDone = true;
}

// Litte tools for debugging/monitoring

static void uuxDebugPinSetup(void)
{
    gpio_config_t LEDgpio;

#ifdef STM32F10X
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif
#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
#endif

    LEDgpio.pin = Pin_4;
    LEDgpio.speed = Speed_2MHz;
    LEDgpio.mode = Mode_Out_PP;
    gpioInit(GPIOB, &LEDgpio);
    digitalLo(GPIOB, Pin_4);

    gpio_config_t ZEDgpio;
    ZEDgpio.pin = Pin_5;
    ZEDgpio.speed = Speed_2MHz;
    ZEDgpio.mode = Mode_Out_PP;
    gpioInit(GPIOB, &ZEDgpio);
    digitalLo(GPIOB, Pin_5);
}

static void uuxDebugOFF(void)
{
    digitalLo(GPIOB, Pin_4);
}

static void uuxDebugON(void)
{
    digitalHi(GPIOB, Pin_4);
}

static void uuxZEDOFF(void)
{
    digitalLo(GPIOB, Pin_5);
}

static void uuxZEDON(void)
{
    digitalHi(GPIOB, Pin_5);
}

/*
 * Port level
 */
// XXX Rely on probe routine for FLASH/GYRO chips for now
// XXX Proper handling for shared SPI cases...
static void uuxInitSPI(uuxSerial_t *uux)
{
    UNUSED(uux);
}

/*
 * Register level access
 */
#define NXP_SUBADDR(chan,reg) (((reg) << 3)|((chan)<< 1))

#define nxpReadI2C(addr, chan, reg, n, buf) \
            i2cRead(addr, NXP_SUBADDR(chan, reg), n, buf)
#define nxpWriteI2C(addr, chan, reg, data) \
            i2cWrite(addr, NXP_SUBADDR(chan, reg), data)
#define nxpWriteBufferI2C(addr, chan, reg, n, buf) \
            i2cWriteBuffer(addr, NXP_SUBADDR(chan, reg), n, buf)

static
void
nxpReadSPI(uuxSerial_t *uux, uint8_t reg, uint8_t *buf)
{
    ENABLE_NXP(uux);
    spiTransferByte(uux->spibus, 0x80|NXP_SUBADDR(uux->chan, reg));
    *buf = spiTransferByte(uux->spibus, 0);
    DISABLE_NXP(uux);
}

static
void
nxpReadBufferSPI(uuxSerial_t *uux, uint8_t reg, int len, uint8_t *buf)
{
    ENABLE_NXP(uux);
    spiTransferByte(uux->spibus, 0x80|(reg << 3));
    spiTransfer(uux->spibus, buf, NULL, len);
    DISABLE_NXP(uux);
}

static
void
nxpWriteSPI(uuxSerial_t *uux, uint8_t reg, uint8_t val)
{
    uint8_t wbuf[2];

    wbuf[0] = NXP_SUBADDR(uux->chan, reg);
    wbuf[1] = val;
    ENABLE_NXP(uux);
    spiTransfer(uux->spibus, NULL, wbuf, 2);
    DISABLE_NXP(uux);
}

static
void
nxpWriteBufferSPI(uuxSerial_t *uux, uint8_t reg, int len, uint8_t *buf)
{
    ENABLE_NXP(uux);
    spiTransferByte(uux->spibus, reg);
    spiTransfer(uux->spibus, NULL, buf, len);
    DISABLE_NXP(uux);
}

static bool
nxpRead(uuxSerial_t *uux, uint8_t reg, int n, uint8_t *buf)
{
    if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
        uux->bcycle += n + 5;
    	return nxpReadI2C(uux->addr, uux->chan, reg, n, buf);
    } else {
        if (n == 1)
            nxpReadSPI(uux, reg, buf);
        else
            nxpReadBufferSPI(uux, reg, n, buf);

        return true;
    }
}

static bool
nxpWrite(uuxSerial_t *uux, uint8_t reg, uint8_t data)
{
    if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
        uux->bcycle += 3;
        return nxpWriteI2C(uux->addr, uux->chan, reg, data);
    } else {
        nxpWriteSPI(uux, reg, data);
        return true;
    }
}

static bool
nxpWriteBuffer(uuxSerial_t *uux, uint8_t reg, int n, uint8_t *buf)
{
    if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
        uux->bcycle += n + 2;
        return nxpWriteBufferI2C(uux->addr, uux->chan, reg, n, buf);
    } else {
        nxpWriteBufferSPI(uux, reg, n, buf);
        return true;
    }
}

static
bool
uuxResetNXP(uuxSerial_t *uux)
{
    uint8_t ioc, lcr, lsr, efcr;

#if 0
    if (uux->bustype == UUXSERIAL_BUSTYPE_SPI) {
        // Software reset in SPI mode is broken?
        // XXX Needs further investigation.
        dprintf(("nxpReset: SPI, assume successful reset\r\n"));
        goto resetok;
    }
#endif

#if 1
uint8_t spr;
nxpRead(uux, IS7x0_REG_SPR, 1, &spr);
dprintf(("nxpReset: pre reset spr 0x%x\r\n", spr));
#endif

    nxpRead(uux, IS7x0_REG_IOCONTROL, 1, &ioc);
    nxpWrite(uux, IS7x0_REG_IOCONTROL, ioc|IS7x0_IOC_SRESET);

    for (int retry = 0 ; retry < 10 ; retry++) {
        delay(5);  // This is a software reset, so delay can be much shorter???.
        nxpRead(uux, IS7x0_REG_IOCONTROL, 1, &ioc);
        if (ioc & IS7x0_IOC_SRESET) {
            dprintf(("nxpReset: ioc 0x%x\r\n", ioc));
        }
    }

    if (ioc & IS7x0_IOC_SRESET) {
        dprintf(("nxpReset: SRESET didn't go down ioc 0x%x\r\n", ioc));
        return false;
    }

#if 1
nxpRead(uux, IS7x0_REG_SPR, 1, &spr);
dprintf(("nxpReset: post reset spr 0x%x\r\n", spr));
#endif

    if (!nxpRead(uux, IS7x0_REG_LCR, 1, &lcr)
     || !nxpRead(uux, IS7x0_REG_LSR, 1, &lsr)) {
        dprintf(("nxpReset: lcr or lcr read failed\r\n"));
        return false;
    }

    dprintf(("nxpReset: lcr 0x%x lsr 0x%x\r\n", lcr, lsr));

    if (lcr == 0x1D && (lsr & 0x60) == 0x60) {
        goto resetok;
    }

    return false;

resetok:;
    // Disable transmitter & receiver
    nxpRead(uux, IS7x0_REG_EFCR, 1, &efcr);
    nxpWrite(uux, IS7x0_REG_EFCR, efcr|IS7x0_EFCR_TXDISABLE|IS7x0_EFCR_RXDISABLE);
    return true;
}

static
bool
uuxResetUB(uuxSerial_t *uux)
{
    UNUSED(uux);
    return true;
}

static
bool
uuxResetUBLOX(uuxSerial_t *uux)
{
    UNUSED(uux);
    return true;
}

static
bool
uuxReset(uuxSerial_t *uux)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_NXP)
        return uuxResetNXP(uux);
    else if (uux->devtype == UUXSERIAL_DEVTYPE_MINIMAL)
        return uuxResetUB(uux);
    else if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return uuxResetUBLOX(uux);

    return false;
}

static
bool
uuxProbeNXP(uuxSerial_t *uux)
{
    uint8_t lcr, lsr;
    uint8_t txlvl;

    if (!nxpRead(uux, IS7x0_REG_LCR, 1, &lcr)
     || !nxpRead(uux, IS7x0_REG_LSR, 1, &lsr)) {
        dprintf(("uuxProbeNXP: lcr or lsr read failed\r\n"));
        return false;
    }

    dprintf(("uuxProbeNXP: lcr 0x%x lsr 0x%x\r\n", lcr, lsr));

    uint8_t iir;
    nxpRead(uux, IS7x0_REG_IIR, 1, &iir);
    dprintf(("uuxProbeNXP: iir 0x%x\r\n", iir));

    // Reset state: LCR = 0x1D, LSR = 0x60
    // "7.4.1 Hardware reset, Power-On Reset (POR) and software reset"
    if (lcr == 0x1D && (lsr & 0x60) == 0x60) {
        dprintf(("uuxProbeNXP: lcr and lsr reset signature match\r\n"));
        return true;
    }

    // Not after reset or unknown chip.
    // Try reading the SPR for our signature.
    uint8_t spr;
    nxpRead(uux, IS7x0_REG_SPR, 1, &spr);
    dprintf(("uuxProbeNXP: spr 0x%x\r\n", spr));

    if (spr == 'U') {
        // It looks like we have seen this chip before.
        dprintf(("uuxProbeNXP: spr signature match\r\n"));
        return true;
    }

    // Try identifying the chip non-intrusively.

    // Wait until THR and TSR to go empty.

    // o If TXLVL is not zero, see if TXLVL is decreasing.
    //   (Must wait for longest 1 char time = 1/15 sec)

    if (!(lsr & IS7x0_LSR_TXEMPTY)) {
            // XXX Should be long enough for 64 bytes to go out @150bps
            // XXX 150/10 = 15 bytes/sec, 64/15 = 4.267secs!!!
            // 100msec is good for 1char@9600bps
            delay(100);

            nxpRead(uux, IS7x0_REG_LSR, 1, &lsr);
            if (!(lsr & IS7x0_LSR_TXEMPTY))
                return false;
    }

    // TX is all empty, TX FIFO Level should be 0x40 (64)

    nxpRead(uux, IS7x0_REG_TXLVL, 1, &txlvl);

    if (txlvl == 64) {
        dprintf(("uuxProbeNXP: txlvl signature match\r\n"));
        return true;
    }

    return false;
}

static
bool
uuxProbeUB(uuxSerial_t *uux)
{
    uint8_t id[4];

    // Arduino based bridge takes a bit to boot
    // XXX Investigate accurate timing
    delay(200);

    if (!nxpRead(uux, IS7x0_REG_SPR, 4, id))
        return false;

    if (id[0] == 'U' && id[1] == 'B') {
        uux->apiver = (id[2] << 8)|id[3];
        return true;
    } else if (id[0] == 'U' && id[1] == 'U') {
        // PIC temporary, should implement "UBxx".
        uux->apiver = 99;
        return true;
    }

    return false;
}

static
bool
uuxProbeUBLOX(uuxSerial_t *uux)
{
    // Observations and assumptions:
    // (1) All data registers except 0xfd (length high), 0xfe (length low) and
    // 0xff (stream outlet) returns their address.
    // (2) Length is limited to relatively small number and does not extend to
    // 0xfd00~ range (On MAX-M8Q, only goes up to 0x19xx.)
    //
    // Probe strategy:
    // (1) Read reg 0xf0 to 0xff.
    // (2) Check if 0xf0 to 0xfc are read as their address, and
    // (3) Check if 0xfd != 0xfd

    uint8_t ubuf[14];

    if (!i2cRead(uux->addr, 0xf0, 14, ubuf))
        return false;

    for (int i = 0 ; i <= 0xc ; i++) {
        if (ubuf[i] != 0xf0 + i)
            return false;
    }

    if (ubuf[0xd] >= 0xfd)
        return false;

#if 0
// A little test for MAX-M8Q
int retry = 0;
int mlen;
uint8_t lbuf[2];
uint8_t dbuf[16];
int tlen;
bool ok;

do {
  if (retry) delay(1000);
  do {
    ok = i2cRead(uux->addr, 0xfd, 2, lbuf);
  } while (!ok);
  mlen = (lbuf[0] << 0)|lbuf[1];
} while (mlen == 0 && ++retry < 10);

// Dump out the garbage
while (mlen) {
  tlen = (mlen > 16) ? 16 : mlen;
  i2cRead(uux->addr, 0xff, tlen, dbuf);
}

// Wait for a new message
retry = 0;

do {
  if (retry) delay(1000);
  do {
    ok = i2cRead(uux->addr, 0xfd, 2, lbuf);
  } while (!ok);
  mlen = (lbuf[0] << 0)|lbuf[1];
} while (mlen == 0 && ++retry < 10);

// Read it
while (mlen) {
  tlen = (mlen > 16) ? 16 : mlen;
  i2cRead(uux->addr, 0xff, tlen, dbuf);
}
#endif

    return true;
}

static
bool
uuxProbe(uuxSerial_t *uux)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_NXP)
        return uuxProbeNXP(uux);
    else if (uux->devtype == UUXSERIAL_DEVTYPE_MINIMAL)
        return uuxProbeUB(uux);
    else if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return uuxProbeUBLOX(uux);

    return false;
}

// UB emulators takes baudrate / 150 as input to BRH and BRL
static void nxpSetSpeedUB(uuxSerial_t *uux)
{
    uint8_t brreg;

    brreg = uux->port.baudRate / 150;
    nxpWrite(uux, UB_REG_BRL, brreg & 0xff);
    UBdelayMicroseconds(100);
    nxpWrite(uux, UB_REG_BRH, (brreg >> 8) & 0xff);
    return;
}

static void nxpSetSpeed(uuxSerial_t *uux)
{
    uint32_t divisor;
    uint32_t prescaler;
    uint32_t baudrate;

    if (uux->freq == UB_FREQ_DONTCARE)
        return;  // Fixed or don't care

    baudrate = uux->port.baudRate;

    if (uux->freq == UB_FREQ_BAUDx150) {
        nxpSetSpeedUB(uux);
        return;
    }

#if 0
#define MAX_BAUDRATE 57600
    if (baudrate > MAX_BAUDRATE)
        baudrate = MAX_BAUDRATE;
#endif

    prescaler= 1;
    divisor = (uux->freq/prescaler)/(baudrate * 16);

    if (divisor > 65535) {
        prescaler = 4;
        divisor = (uux->freq/prescaler)/(baudrate * 16);
    }

    // Set the new prescaler if needed
    uint8_t mcr;
    nxpRead(uux, IS7x0_REG_MCR, 1, &mcr);

    if (mcr & IS7x0_MCR_CLKDIV) {
        if (prescaler == 1) {
            mcr &= ~(1 << 7);
            nxpWrite(uux, IS7x0_REG_MCR, mcr);
        }
    } else {
        if (prescaler == 4) {
            mcr |= (1 << 7);
            nxpWrite(uux, IS7x0_REG_MCR, mcr);
        }
    }

    // Enable access to divisor latches (DLL & DLH)
    uint8_t lcr;
    nxpRead(uux, IS7x0_REG_LCR, 1, &lcr);
    nxpWrite(uux, IS7x0_REG_LCR, lcr|IS7x0_LCR_DIVLATEN);

    // Write the new divisor

    nxpWrite(uux, IS7x0_REG_DLL, divisor & 0xff);
    nxpWrite(uux, IS7x0_REG_DLH, (divisor >> 8) & 0xff);

    // Restore access to LCR

    nxpWrite(uux, IS7x0_REG_LCR, lcr);
}

static void nxpSetLine(uuxSerial_t *uux)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_MINIMAL
     || uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    portOptions_t options = uux->port.options; 
    uint8_t lcr = IS7x0_LCR_WLEN8;

    if (options & SERIAL_STOPBITS_2)
        lcr |= IS7x0_LCR_STOP2;

    if (options & SERIAL_PARITY_EVEN)
        lcr |= IS7x0_LCR_PAREVEN;

    nxpWrite(uux, IS7x0_REG_LCR, lcr);
}

static void nxpEnableTransmitter(uuxSerial_t *uux, bool enable)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    if (enable)
        uux->efcr &= ~IS7x0_EFCR_TXDISABLE;
    else
        uux->efcr |= IS7x0_EFCR_TXDISABLE;

    nxpWrite(uux, IS7x0_REG_EFCR, uux->efcr);
    UBdelayMicroseconds(50);
}

static void nxpEnableReceiver(uuxSerial_t *uux, bool enable)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    if (enable)
        uux->efcr &= ~IS7x0_EFCR_RXDISABLE;
    else
        uux->efcr |= IS7x0_EFCR_RXDISABLE;

    nxpWrite(uux, IS7x0_REG_EFCR, uux->efcr);
    UBdelayMicroseconds(50);
}

static void nxpEnableEnhancedFunctions(uuxSerial_t *uux)
{
    uint8_t lcr;
    uint8_t efr;

    if (uux->devtype == UUXSERIAL_DEVTYPE_MINIMAL
     || uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    nxpRead(uux, IS7x0_REG_LCR, 1, &lcr);
    nxpWrite(uux, IS7x0_REG_LCR, 0xbf);

    nxpRead(uux, IS7x0_REG_EFR, 1, &efr);
    efr |= IS7x0_EFR_ENH;
    nxpWrite(uux, IS7x0_REG_EFR, efr);

    nxpWrite(uux, IS7x0_REG_LCR, lcr);
}

static void nxpFIFOEnable(uuxSerial_t *uux)
{
    uint8_t fcr;

    if (uux->devtype == UUXSERIAL_DEVTYPE_MINIMAL
     || uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    //nxpRead(uux, IS7x0_REG_FCR, 1, &fcr);

    fcr = IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST|IS7x0_FCR_FIFO_EN;

    nxpWrite(uux, IS7x0_REG_FCR, fcr);

    //uux->fcr = fcr;
}

static void nxpSetTriggerLevel(uuxSerial_t *uux)
{
    uint8_t fcr;
    uint8_t mcr;
    uint8_t tlr;

    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX)
        return;

    nxpRead(uux, IS7x0_REG_FCR, 1, &fcr);
    fcr = IS7x0_FCR_TXFIFO_RST|IS7x0_FCR_RXFIFO_RST; // RX and TX triggers to zero, reset both fifos, disable FIFO.
    nxpWrite(uux, IS7x0_REG_FCR, fcr);
    UBdelayMicroseconds(50);

#if 0
uint8_t spr;
nxpRead(uux, IS7x0_REG_SPR, 1, &spr);
printf("nxpSetTriggerLevel: pre spr 0x%x\r\n", spr);
#endif

    // Enable TCR/TLR access
    nxpRead(uux, IS7x0_REG_MCR, 1, &mcr);
    mcr |= IS7x0_MCR_TCRTLR_EN;
    nxpWrite(uux, IS7x0_REG_MCR, mcr);
    UBdelayMicroseconds(50);

    tlr = (1 << IS7x0_TLR_RX_SFT)|(((UUXSERIAL_I2C_MAX_TXFRAG + 1) / 4) << IS7x0_TLR_TX_SFT);
    nxpWrite(uux, IS7x0_REG_TLR, tlr);

    // Disable TCR/TLR access
    mcr &= ~IS7x0_MCR_TCRTLR_EN;
    nxpWrite(uux, IS7x0_REG_MCR, mcr);
    UBdelayMicroseconds(50);

#if 0
nxpRead(uux, IS7x0_REG_SPR, 1, &spr);
printf("nxpSetTriggerLevel: post spr 0x%x\r\n", spr);
#endif

#if 0 // Done in nxpFIFOEnable()
    fcr = IS7x0_FCR_FIFO_EN;
    nxpWrite(uux, IS7x0_REG_FCR, fcr);
#endif
}

static void nxpEnableInterrupt(uuxSerial_t *uux)
{
    uint8_t ier;

    if (uux->polled)
        return;

    ier = IS7x0_IER_RHR;
    //ier |= IS7x0_IER_THR;
    //ier |= IS7x0_IER_CTS;
    //ier |= IS7x0_IER_RTS;
    //ier |= IS7x0_IER_MODEM;
    //ier |= IS7x0_IER_LINE;

    nxpWrite(uux, IS7x0_REG_IER, ier);
}

serialPort_t *openUUXSerial(
	uuxSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback,
	uint32_t baud, portOptions_t options)
{
    uuxSerial_t *uux;

    if (uuxSerialPortsInit) {
        for (int i = 0 ; i < MAX_UUXSERIAL_PORTS; i++)
            uuxSerialPorts[i].active = false;
        uuxSerialPortsInit = true;
    }

    uux = &uuxSerialPorts[portIndex];

    switch (portIndex) {
    case 7:
        // Sparkfun BOB on SPI
        uux->bustype = UUXSERIAL_BUSTYPE_SPI;
        uux->spibus = UUXSERIAL_SPI_INSTANCE;
        uux->spicsgpio = UUXSERIAL_SPI_CS_GPIO;
        uux->spicspin = UUXSERIAL_SPI_CS_PIN;
        uux->devtype = UUXSERIAL_DEVTYPE_NXP;
        uux->freq = 14745600;
        uux->polled = 1;

        uux->devops.reset = uuxReset;
        break;

    case 0:
        // Sparkfun BOB
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_NXP;
        uux->addr = 0x4d;
        uux->chan = 0;
        uux->freq = 14745600;
        uux->polled = 1;

        //setupDebugPins();

        break;

    case 1:
        // Switch Science BOB
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_NXP;
        uux->addr = 0x4c;
        uux->chan = 0;
        uux->freq = 12000000;
        uux->polled = 1;
        break;

    case 2:
        // pic-twub (1825)
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_MINIMAL; // MINIMAL, MINIMAL_IMUX
        uux->addr = 0x36;
        uux->chan = 0;
        uux->freq = UB_FREQ_BAUDx150;
        uux->polled = 1;
        break;

    case 3:
        // pic-twub (1822)
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_MINIMAL;
        uux->addr = 0x38;
        uux->chan = 0;
        uux->freq = UB_FREQ_BAUDx150;
        uux->polled = 1;
        break;

    case 4:
        // MWOSD
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_MINIMAL;
        uux->addr = 0x19;
        uux->chan = 0;
        uux->freq = UB_FREQ_BAUDx150;
        uux->polled = 1;
        break;

    case 5:
        // Arduino Pro Mini UB
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_MINIMAL;
        uux->addr = 0x18;
        uux->chan = 0;
        uux->freq = UB_FREQ_BAUDx150;
        uux->polled = 1;
        break;

    case 6:
        // u-blox DDC
        uux->bustype = UUXSERIAL_BUSTYPE_I2C;
        uux->devtype = UUXSERIAL_DEVTYPE_UBLOX;
        uux->addr = 0x42;
        uux->freq = -1;
        uux->polled = 1;
        break;


    default:
        return NULL;
    }

    if (uux->bustype == UUXSERIAL_BUSTYPE_SPI) {
        spiInitSettings(uux->spibus, &uux->spisettings, 0, SPI_4MHZ_CLOCK_DIVIDER);
    }

    dprintf(("openNXPSerial: probing and resetting portIndex %d\r\n", portIndex));

    if (!uuxProbe(uux)) {
        dprintf(("openNXPSerial: probe failed\r\n"));
    }

    // If this is a NXP chip, we write a signature into SPR for hot start.
    if (uux->devtype == UUXSERIAL_DEVTYPE_NXP)
        nxpWrite(uux, IS7x0_REG_SPR, 0x55);

    if (!uuxReset(uux)) {
        if (uux->devtype == UUXSERIAL_DEVTYPE_NXP
          && uux->bustype == UUXSERIAL_BUSTYPE_SPI) {
            dprintf(("openNXPSerial: reset failed for NXP/SPI, continuing\r\n"));
        } else {
            dprintf(("openNXPSerial: reset failed\r\n"));
            return NULL;
        }
    }

    uux->port.vTable = uuxSerialVTable;
    uux->port.baudRate = baud;
    uux->port.mode = MODE_RXTX;
    uux->port.options = options;
    uux->port.callback = callback;
    uux->uuxSerialPortIndex = portIndex;

    resetBuffers(uux);

    uux->rxlvl = 0;
    uux->txlvl = 0; // Actual value will be read for the 1st TX data

    uux->bcycle = 0;

    if (uux->devtype != UUXSERIAL_DEVTYPE_UBLOX) {
        // Superfluious reset???
        nxpWrite(uux, IS7x0_REG_IOCONTROL, IS7x0_IOC_SRESET);
        UBdelayMicroseconds(50);

        nxpEnableEnhancedFunctions(uux);
        nxpSetSpeed(uux);
        nxpSetLine(uux);
        nxpSetTriggerLevel(uux);
        nxpFIFOEnable(uux);
        nxpRead(uux, IS7x0_REG_EFCR, 1, &uux->efcr);
    }

    //EXTI_INIT();
    uuxSerialIntExtiInit();

    nxpEnableInterrupt(uux);

    uux->active = true;

    return &uux->port;
}

void uuxSerialWriteByte(serialPort_t *instance, uint8_t ch)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    if (!uux->active)
        return;

    if (txBufferRoom(uux->port)) {
        uux->port.txBuffer[uux->port.txBufferHead] = ch;
        uux->port.txBufferHead = (uux->port.txBufferHead + 1)
                % uux->port.txBufferSize;
    }
}

uint8_t uuxSerialRxBytesWaiting(serialPort_t *instance)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    if (!uux->active)
        return 0;

    return rxBufferLen(uux->port);
}

uint8_t uuxSerialTxBytesFree(serialPort_t *instance)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    if (!uux->active)
        return 0;

    return txBufferRoom(uux->port);
}

uint8_t uuxSerialReadByte(serialPort_t *instance)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;
    int rxavail;
    uint8_t ch;

    if (!uux->active)
        return 0;

    rxavail = rxBufferLen(uux->port);

    if (rxavail) {
        ch = uux->port.rxBuffer[uux->port.rxBufferTail];
        uux->port.rxBufferTail = (uux->port.rxBufferTail + 1)
                % uux->port.rxBufferSize;
        return ch;
    }
    return 0;
}

void uuxSerialSetBaudRate(serialPort_t *instance, uint32_t baudrate)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    uux->port.baudRate = baudrate;
    nxpSetSpeed(uux);
}

bool uuxSerialTransmitBufferEmpty(serialPort_t *instance)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    if (!uux->active)
        return false;

    return txBufferLen(uux->port) == 0;
}

void uuxSerialSetMode(serialPort_t *instance, portMode_t mode)
{
    uuxSerial_t *uux = (uuxSerial_t *)instance;

    if (!uux->active)
        return;

    uux->port.mode = mode;

    nxpEnableTransmitter(uux, ((uux->port.mode & MODE_TX) == MODE_TX));
    nxpEnableReceiver(uux, ((uux->port.mode & MODE_RX) == MODE_RX));
}

static void uuxUpdateRXLVL(uuxSerial_t *uux)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX) {
        // u-blox's data queue is big, but data can be read in small chunks.
        uint8_t lbuf[2];
        uint16_t len; 
        if (!i2cRead(uux->addr, UBX_REG_LENHI, 2, lbuf)) {
            // Failed to read RXLVL... just return?
            return;
        }
        len = (lbuf[0] << 8)|lbuf[1];
        uux->rxlvl = (len > 255) ? 255 : len;
    } else {
        nxpRead(uux, IS7x0_REG_RXLVL, 1, &uux->rxlvl);
    }
}

static void uuxUpdateTXLVL(uuxSerial_t *uux)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX) {
        // u-blox doesn't provide any mechanism for write pacing,
        // But since messages can be written in small chunks,
        // pretend it has a reasonable amount of buffer.
        uux->txlvl = 64;
    } else {
        nxpRead(uux, IS7x0_REG_TXLVL, 1, &uux->txlvl);
    }
}

static void uuxReadFifo(uuxSerial_t *uux, int len, uint8_t *buf)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX) {
        i2cRead(uux->addr, UBX_REG_DATA, len, buf);
    } else {
        nxpRead(uux, IS7x0_REG_RHR, len, buf);
    }
}

static void uuxWriteFifo(uuxSerial_t *uux, int len, uint8_t *buf)
{
    if (uux->devtype == UUXSERIAL_DEVTYPE_UBLOX) {
        i2cWriteBuffer(uux->addr, -1, len, buf);
    } else {
        nxpWriteBuffer(uux, IS7x0_REG_THR, len, buf);
    }
}

// XXX Good old printf, should go away.
#include "common/printf.h"

void uuxProcess(void)
{
    int bcycle;

    bool interrupted;
    static int scanport = 0;
    uuxSerial_t *uux;

    int rxroom;
    int rxburst;
    static int rxlen;

    int txavail;
    int txburst;
    int txlen;

    //digitalHi(GPIOB, Pin_4);

#if 0
  // Arduino-UB
  static uint8_t dummytbuf[16];
  static uint8_t dummyrbuf[128];
  static int seq = 0;
  static int dummycount = 0;

  if ((++dummycount % 10) == 0) {
    tfp_sprintf(dummytbuf, "%d\r\n", seq++);
    if (seq > 9999) seq = 0;
    i2cWriteBuffer(0x19, IS7x0_REG_THR, 6, dummytbuf);

    delayMicroseconds(40);

    i2cRead(0x19, IS7x0_REG_RXLVL, 1, &rxqlen);

    if (rxqlen > 0) {
        rxlen = rxqlen > 16 ? 16 : rxqlen;
        i2cRead(0x19, IS7x0_REG_RHR, rxlen, dummyrbuf);
    }
  }
#endif

    __disable_irq();
    interrupted = nxpInterrupted;
    nxpInterrupted = false;
    __enable_irq();

    // Quickly scan the slaves for interrupts

    for (int i = 0 ; i < MAX_UUXSERIAL_PORTS ; i++) {
        uux = &uuxSerialPorts[i];

        if (!uux->active)
            continue;

        uux->bcycle = 0;

        if (interrupted)
            nxpRead(uux, IS7x0_REG_IIR, 1, &uux->iir);
        else if (uux->polled)
            // Simulate RX time-out
            uux->iir = IS7x0_IIR_RXTIMO;
    }

    bcycle = 0;

    for (int i = 0 ; i < MAX_UUXSERIAL_PORTS ; i++) {

        if (bcycle > 32)
            break;

        uux = &uuxSerialPorts[scanport];
        scanport = (scanport + 1) % MAX_UUXSERIAL_PORTS;

        if (!uux->active)
            continue;

        if ((uux->iir & IS7x0_IIR_INTSTAT)
         && (uux->rxlvl == 0)
         && (txBufferLen(uux->port) == 0))
            goto out;

        switch (uux->iir & IS7x0_IIR_INTMSK) {
        case IS7x0_IIR_RXTIMO:   // RX chars below trigger are available
        case IS7x0_IIR_RHR:      // RX chars above trigger are available
            // Update the rxlvl
            uuxUpdateRXLVL(uux);
            break;

        case IS7x0_IIR_THR:      // TX fifo ready for more queuing
            // Update the txlvl
            uuxUpdateTXLVL(uux);
            break;

        case IS7x0_IIR_LINESTAT:
            // Hard case here: there is at least one char
            // with line error, but we don't know which.
            // To salvage valid chars, we have to read chars in the
            // fifo one by one with associated LSR, which is a pain.
            // If rxlvl is non-zero (left over from last service),
            // we can assume #rxlvl chars are error free.

            // For now, just reset the RX FIFO, for KISS.
            // We can handle the safe #rxlvl chars when the code is mature.
            nxpWrite(uux, IS7x0_REG_FCR, uux->fcr & ~IS7x0_FCR_TXFIFO_RST);
            UBdelayMicroseconds(50); // XXX
            nxpRead(uux, IS7x0_REG_RXLVL, 1, &uux->rxlvl);
            nxpRead(uux, IS7x0_REG_IIR, 1, &uux->iir);
            break;

        default: 
            break;
        }

        // If the current device is in MODE_RX and
        // if there's a room in rxBuf, try to read from RX FIFO.
        // Limit single transfer to MIN(rxroom, rxfifolevel, maxfrag).
        // Actual transfer size will further be limited by the end
        // of the physical buffer (can't wrap around in a burst transfer).

        if (uux->port.mode & MODE_RX) {

            rxroom = rxBufferRoom(uux->port);

            if (rxroom) {
                if (uux->rxlvl) {
                    rxlen = (rxroom < uux->rxlvl) ? rxroom : uux->rxlvl;

                    if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
                        if (rxlen > UUXSERIAL_I2C_MAX_RXFRAG)
                            rxlen = UUXSERIAL_I2C_MAX_RXFRAG;
                    } else {
                        if (rxlen > UUXSERIAL_SPI_MAX_RXFRAG)
                            rxlen = UUXSERIAL_SPI_MAX_RXFRAG;
                    }

                    rxburst = rxBufferBurstLimit(uux->port);

                    if (rxlen > rxburst)
                        rxlen = rxburst;

                    // Debuggin' (Shouldn't happen)
                    if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
                        if (rxlen <= 0 || rxlen > UUXSERIAL_I2C_MAX_RXFRAG) {
                            rxlen = 1;
                        }
                    } else {
                        if (rxlen <= 0 || rxlen > UUXSERIAL_SPI_MAX_RXFRAG) {
                            rxlen = 1;
                        }
                    }

                    // XXX Cast (uint8_t *) is required to cancel volatile...
                    uuxReadFifo(uux, rxlen, 
                            (uint8_t *)&(uux->port.rxBuffer[uux->port.rxBufferHead]));

                    uux->port.rxBufferHead = (uux->port.rxBufferHead + rxlen)
                            % uux->port.rxBufferSize;

                    uux->rxlvl -= rxlen;
                }
            }

            // If callback is set, feed newly received data.
            int rxavail;

            if (uux->port.callback && (rxavail = rxBufferLen(uux->port))) {
                uint8_t ch;
                while (rxavail--) {
                    ch = uux->port.rxBuffer[uux->port.rxBufferTail];
                    uux->port.rxBufferTail = (uux->port.rxBufferTail + 1)
                            % uux->port.rxBufferSize;
                    (*uux->port.callback)(ch);
                }
            }
        }

        // XXX Take rxlen into account for txlen calculation.

        if ((uux->port.mode & MODE_TX)
            && ((txavail = txBufferLen(uux->port)) != 0)
            && bcycle < 32) {

            // Can pump out without retrieving a new txlvl value,
            // knowing there is at least old txlvl bytes of space
            // available for queuing.

            if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
                if (uux->txlvl < UUXSERIAL_I2C_MAX_TXFRAG)
                    nxpRead(uux, IS7x0_REG_TXLVL, 1, &uux->txlvl);
            } else {
                if (uux->txlvl < UUXSERIAL_SPI_MAX_TXFRAG)
                    nxpRead(uux, IS7x0_REG_TXLVL, 1, &uux->txlvl);
            }

            txburst = txBufferBurstLimit(uux->port);

            if (uux->txlvl) {
                txlen = (txavail < uux->txlvl) ? txavail : uux->txlvl;

                if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
                    if (txlen > UUXSERIAL_I2C_MAX_TXFRAG)
                        txlen = UUXSERIAL_I2C_MAX_TXFRAG;
                } else {
                    if (txlen > UUXSERIAL_SPI_MAX_TXFRAG)
                        txlen = UUXSERIAL_SPI_MAX_TXFRAG;
                }

                if (txlen > txburst)
                    txlen = txburst;

                if (uux->bustype == UUXSERIAL_BUSTYPE_I2C) {
	            if (txlen <= 0 || txlen > UUXSERIAL_I2C_MAX_TXFRAG) {
                        // debug[3] = txlen;
                        txlen = 1;
                    }
                } else {
	            if (txlen <= 0 || txlen > UUXSERIAL_SPI_MAX_TXFRAG) {
                        // debug[3] = txlen;
                        txlen = 1;
                    }
                }

                // XXX Cast (uint8_t *) is required to cancel volatile...
                uuxWriteFifo(uux, txlen, 
                        (uint8_t *)&(uux->port.txBuffer[uux->port.txBufferTail]));

                uux->port.txBufferTail = (uux->port.txBufferTail + txlen)
                        % uux->port.txBufferSize;

                uux->txlvl -= txlen;
            }
        }

out:;
        if (!uux->polled)
            nxpRead(uux, IS7x0_REG_IIR, 1, &uux->iir);

        int index = uux - uuxSerialPorts;
        if (index < 2) {
        	if (uux->bcycle > debug[index])
			debug[index] = uux->bcycle;
        }

        bcycle += uux->bcycle;
    }

    if (bcycle > debug[2])
        debug[2] = bcycle;

    //digitalLo(GPIOB, Pin_4);
}

const struct serialPortVTable uuxSerialVTable[] = {
    {
        uuxSerialWriteByte,
        uuxSerialRxBytesWaiting,
        uuxSerialTxBytesFree,
        uuxSerialReadByte,
        uuxSerialSetBaudRate,
        uuxSerialTransmitBufferEmpty,
        uuxSerialSetMode,
        .writeBuf = NULL,
    }
};
