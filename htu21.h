#ifndef _HTU21_H_
#define _HTU21_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/** Default I2C address for the HTU21D. */
#define HTU21_I2CADDR         (0x40)

/** Read temperature register. */
#define HTU21_READTEMP        (0xE3)

/** Read humidity register. */
#define HTU21_READHUM         (0xE5)

/** Write register command. */
#define HTU21_WRITEREG        (0xE6)

/** Read register command. */
#define HTU21_READREG         (0xE7)

/** Reset command. */
#define HTU21_RESET           (0xFE)

/* Sets up the HTU21D. Must have configured i2c_master.h */
bool htu21_init(void);

/* Returns integer Temperature x 100 */
uint32_t htu21_readtemp(void);

/* Returns integer Humidity x 100 */
uint32_t htu21_readhumidity(void);

#endif // _HTU21_H_
