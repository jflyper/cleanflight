#ifdef VTX_COMMON_DPRINTF
#include "common/printf.h"
#include "drivers/system.h"
#include "io/serial.h"
serialPort_t *debugSerialPort;
# define dprintf(x) if (debugSerialPort) printf x
#else // VTX_COMMON_DPRINTF
# define dprintf(x)
#endif // VTX_COMMON_DPRINTF
