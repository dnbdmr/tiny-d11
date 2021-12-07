/* PCF8566 I2C LCD driver library */

#ifndef _PCF8566_H_
#define _PCF8566_H_

#include <stdbool.h>
#include <stdint.h>

#define PCF8566_ADDRESS	62

/* Initalize I2C and LCD with sane defaults */
bool lcd_init(void);

/* Push string into buffer and send to LCD.
 * Strips trailing newline. */
bool lcd_write_str(const char *s);

/* Write raw binary to LCD, bits 0-30 */
bool lcd_write_raw(uint64_t data);

/* Push character into end of buffer.
 * Must run lcd_update to send to LCD. */
void lcd_push_char(char c);

/* Send buffer to LCD */
bool lcd_update(void);

/* Set bank to write to and read from */
bool lcd_bank_select(bool in, bool out);

/* Mode Register
 * lp: low power
 * en: enable
 * bias: 0(1/3), 1(1/2)
 * mode: 0(1:4), 1(static), 2(1:2), 3(1:3) */
bool lcd_mode_set(bool lp, bool en, bool bias, uint8_t mode);

/* Blink Register
 * rate: 0(disable), 1(fast) - 3(slow)
 * alt: Use alternae register */
bool lcd_blink_set(uint8_t rate, bool alt);

#endif // _PCF8566_H_

