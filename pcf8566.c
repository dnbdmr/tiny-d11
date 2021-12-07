#include <string.h>
#include "pcf8566.h"
#include "i2c_master.h"

#define REG_MODE_BASE	0b01000000 
#define REG_BLINK_BASE	0b01110000 
#define REG_BANK_BASE	0b01111000 

/* If changing NUM_DIGITS, must edit these defines,
 * and lcd_fontbuf2raw */
#define NUM_DIGITS	4
#define DIG_4	00
#define DIG_3	07
#define DIG_2	15
#define	DIG_1	23

// Function Prototypes -----------------------------------
static void lcd_str2fontbuf(const char *);
static uint64_t lcd_fontbuf2raw(uint8_t buf[4]);
static uint8_t reverse_bits(uint8_t byte);

// Static Data -------------------------------------------
static const uint8_t sevensegfonttable[];
static char input_raw[NUM_DIGITS*2 + 1] = {0};
static uint8_t fontbuf[NUM_DIGITS] = {0};

// Implementation ----------------------------------------

bool lcd_init(void)
{
	i2c_init(100000);
	bool success = i2c_start(PCF8566_ADDRESS<<1);
	if (success) success &= i2c_write_byte(0b11001001); //	Mode Set
	if (success) success &= i2c_write_byte(0b10000000); //	Data Load pointer
	if (success) success &= i2c_write_byte(0b11100000);	//	Device Select
	if (success) success &= i2c_write_byte(0b11111000);	//	Bank Select
	if (success) success &= i2c_write_byte(0b01110000);	//	Blink
	i2c_stop();

	return success;
}

bool lcd_write_str(const char *s)
{
	while ((*s != '\n') && (*s != '\0') && (*s != '\r')) {
		lcd_push_char(*s);
		s++;
	}
	return lcd_update();
}

bool lcd_write_raw(uint64_t data)
{
	bool success = i2c_start(PCF8566_ADDRESS<<1);
	if (success) success &= i2c_write_byte(0b10000000); //	Data Load pointer
	if (success) success &= i2c_write_byte(0b01100000);	//	Device Select

	if (success) success &= i2c_write_byte(reverse_bits(data));
	if (success) success &= i2c_write_byte((reverse_bits(data>>8)) & 0xFF);
	if (success) success &= i2c_write_byte((reverse_bits(data>>16)) & 0xFF);
	if (success) success &= i2c_write_byte((reverse_bits(data>>24)) & 0xFF);
	i2c_stop();

	return success;
}

void lcd_push_char(char c)
{
	uint8_t buflen = strlen(input_raw);
	if (buflen < NUM_DIGITS*2) {
		input_raw[buflen] = c;
		input_raw[buflen+1] = '\0';
	}
	else { // move over by one and add
		while (buflen) {
			input_raw[NUM_DIGITS*2 - buflen] = input_raw[NUM_DIGITS*2 - buflen+1];
			buflen--;
		}
		input_raw[NUM_DIGITS*2-1] = c;
		input_raw[NUM_DIGITS*2] = '\0';
	}
}

bool lcd_update(void)
{
	lcd_str2fontbuf(input_raw);
	return lcd_write_raw(lcd_fontbuf2raw(fontbuf));
}

bool lcd_bank_select(bool in, bool out)
{
	bool success = i2c_start(PCF8566_ADDRESS<<1);
	if (success) success &= i2c_write_byte(REG_BANK_BASE | (in<<1) | (out<<0));	//	Bank Select
	i2c_stop();
	return success;
}

bool lcd_mode_set(bool lp, bool en, bool bias, uint8_t mode)
{
	bool success = i2c_start(PCF8566_ADDRESS<<1);
	if (success) success &= i2c_write_byte(REG_MODE_BASE | (lp << 4) | (en << 3) | (bias << 2) | (mode & 3)); //	Mode Set
	i2c_stop();
	return success;
}

bool lcd_blink_set(uint8_t rate, bool alt)
{
	bool success = i2c_start(PCF8566_ADDRESS<<1);
	if (success) success &= i2c_write_byte(REG_BLINK_BASE | (alt << 2) | (rate & 3));
	i2c_stop();
	return success;

}

static void lcd_str2fontbuf(const char *s)
{
	memset(fontbuf, 0, 4);

	uint8_t len = strlen(s);
	uint8_t bufpos = NUM_DIGITS;
	while (len && bufpos) {
		/* discard if out of array bounds */
		if ((s[len-1] < 32) || (s[len-1] > 95+32)) {
			len--;
			if (!len) break; // Quit if out of characters
		}

		/* if theres a dot add it to the current bufpos and move on */
		if (s[len-1] == '.') {
			fontbuf[bufpos-1] = sevensegfonttable[14]; 
			len--;
			if (!len) break; // Quit if out of characters
			continue;
		}
		fontbuf[bufpos-1] |= sevensegfonttable[s[len-1]-32];
		bufpos--;
		len--;
	}
}

/* Must be changed based on number of digits. */
static uint64_t lcd_fontbuf2raw(uint8_t buf[])
{
	uint64_t rawbuf = 0;
	rawbuf |= buf[0] << DIG_1;
	rawbuf |= buf[1] << DIG_2;
	rawbuf |= buf[2] << DIG_3;
	rawbuf |= (buf[3] & ~(1<<7)) << DIG_4; // clear 8th bit
	return rawbuf;
}

/* PCF8566 reads bytes MSB first */
static uint8_t reverse_bits(uint8_t byte)
{
	return byte = ((byte * 0x0802LU & 0x22110LU) | (byte * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16; 
}

/* From Adafruit:
 * https://github.com/adafruit/Adafruit_LED_Backpack/blob/master/Adafruit_LEDBackpack.cpp
 *
 * Starts at ASCII 0x20(32)
 * Match columns with wiring order
 */
static const uint8_t sevensegfonttable[] = {
//  0bPDEGFABC
	0b00000000, // (space)
	0b10000011, // !
	0b00001010, // "
	0b01111011, // #
	0b01011101, // $
	0b10110010, // %
	0b00010011, // &
	0b00001000, // '
	0b01001100, // (
	0b01000110, // )
	0b00011110, // *
	0b00111000, // +
	0b00100000, // ,
	0b00010000, // -
	0b10000000, // .
	0b00110010, // /
	0b01101111, // 0
	0b00000011, // 1
	0b01110110, // 2
	0b01010111, // 3
	0b00011011, // 4
	0b01011101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01011111, // 9
	0b01000100, // :
	0b01000101, // ;
	0b01110000, // <
	0b01010000, // =
	0b01010001, // >
	0b10110110, // ?
	0b01110111, // @
	0b00111111, // A
	0b01111001, // B
	0b01101100, // C
	0b01110011, // D
	0b01111100, // E
	0b00111100, // F
	0b01101101, // G
	0b00111011, // H
	0b00101000, // I
	0b01100011, // J
	0b00111101, // K
	0b01101000, // L
	0b00100101, // M
	0b00101111, // N
	0b01101111, // O
	0b00111110, // P
	0b01011110, // Q
	0b00101110, // R
	0b01011101, // S
	0b01111000, // T
	0b01101011, // U
	0b01101011, // V
	0b01001010, // W
	0b00111011, // X
	0b01011011, // Y
	0b01110110, // Z
	0b01101100, // [
	0b00011001, //
	0b01000111, // ]
	0b00001110, // ^
	0b01000000, // _
	0b00000010, // `
	0b01110111, // a
	0b01111001, // b
	0b01110000, // c
	0b01110011, // d
	0b01111110, // e
	0b00111100, // f
	0b01011111, // g
	0b00111001, // h
	0b00100000, // i
	0b01000001, // j
	0b00111101, // k
	0b00101000, // l
	0b00100001, // m
	0b00110001, // n
	0b01110001, // o
	0b00111110, // p
	0b00011111, // q
	0b00110000, // r
	0b01011101, // s
	0b01111000, // t
	0b01100001, // u
	0b01100001, // v
	0b00100001, // w
	0b00111011, // x
	0b01011011, // y
	0b01110110, // z
	0b00010011, // {
	0b00101000, // |
	0b00111000, // }
			 0b00000100, // ~
			 0b00000000, // del
};

/* 019700 wiring */
#define SEG_4C	00
#define SEG_4B	01
#define SEG_$A	02
#define SEG_4F	03
#define	SEG_4G	04
#define	SEG_4E	05
#define	SEG_4D	06

#define SEG_3C	07
#define SEG_3B	08
#define	SEG_3A	09
#define SEG_3F	10
#define SEG_3G	11
#define	SEG_3E	12
#define	SEG_3D	13
#define	SEG_DP3	14

#define SEG_2C	15
#define	SEG_2B	16
#define	SEG_2A	17
#define	SEG_2F	18
#define	SEG_2G	19
#define	SEG_2E	20
#define	SEG_2D	21
#define	SEG_DP2	22

#define	SEG_1C	23
#define	SEG_1B	24
#define	SEG_1A	25
#define	SEG_1F	26
#define	SEG_1G	27
#define	SEG_1E	28
#define	SEG_1D	29
#define	SEG_DP1	30
