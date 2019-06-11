#include "htu21.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "samd21.h"
#include "i2c_master.h"

#include "tusb.h"

bool htu21_init(void) 
{
	i2c_init(100000);

	//sent Reset command
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_WRITE))) {
		return false;
	}
	i2c_write_byte(HTU21_RESET);
	i2c_stop();
	
	while(i2c_busy(HTU21_I2CADDR << 1));

	// send ReadReg command
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_WRITE))) {
		return false;
	}
	i2c_write_byte(HTU21_READREG);
	i2c_stop();
	
	//read reg
	uint8_t byte;
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_READ))) {
		return false;
	}
	i2c_read_byte(&byte, 1);
	i2c_stop();

	return (byte == 0x02); //reg must be 0x02, apparently
}

/* Returns integer Temperature x 100 */
uint32_t htu21_readtemp(void) {
	uint8_t byte[3];
	
	// send ReadTemp command
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_WRITE))) {
		return false;
	}
	i2c_write_byte(HTU21_READTEMP);
	// htu21 stretches during conversion
	//
	// repeated start
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_READ))) {
		return 1;
	}
	i2c_read_byte(&byte[0], 0);
	i2c_read_byte(&byte[1], 0);
	i2c_read_byte(&byte[2], 1);
	i2c_stop();

	uint16_t t = byte[0] << 8;
	t |= byte[1] & 0b11111100;
	t = (t * 208/431) - 5233;

	return (uint32_t)t;
}

/* Returns integer Humidity x 100 */
uint32_t htu21_readhumidity(void) {
	uint8_t byte[3];
	
	// send ReadTemp command
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_WRITE))) {
		return false;
	}
	i2c_write_byte(HTU21_READHUM);
	// htu21 stretches during conversion
	//
	// repeated start
	if (!(i2c_start((HTU21_I2CADDR << 1) | I2C_TRANSFER_READ))) {
		return 0.0;
	}
	i2c_read_byte(&byte[0], 0);
	i2c_read_byte(&byte[1], 0);
	i2c_read_byte(&byte[2], 1);
	i2c_stop();

	uint16_t h = byte[0] << 8;
	h |= byte[1] & 0b11111100;

	h = (h * 771 / 4043) - 600;

	return (uint32_t)h;
}
