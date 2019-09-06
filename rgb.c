#include <rgb.h>
#include "hal_gpio.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "samd21.h"

HAL_GPIO_PIN(LED_CK, A, 00);
HAL_GPIO_PIN(LED_DA, A, 01);

void rgb_init(void) {
	HAL_GPIO_LED_CK_out();
	HAL_GPIO_LED_DA_out();
}

void rgb_sendbyte(uint8_t byte) {
    for(uint8_t i=0; i<8; i++) {
        HAL_GPIO_LED_DA_write((byte>>(8-i)) & 0x1);
        HAL_GPIO_LED_CK_set();
        HAL_GPIO_LED_CK_clr();
    }
}

void rgb_update(RGB_type *leds, uint8_t num)
{
    for(uint8_t i=0; i<4; i++){
        rgb_sendbyte(0x00);
    }

    for(uint8_t i=0; i<num; i++) {
        rgb_sendbyte(0xE0 | (leds[i].bright & 0x1F));
        rgb_sendbyte(leds[i].blue);
        rgb_sendbyte(leds[i].green);
        rgb_sendbyte(leds[i].red);
    }

    for(uint8_t i=0; i<4; i++){
        rgb_sendbyte(0xFF);
    }
}

void rgb_zero(uint8_t num)
{
    for(uint8_t i=0; i<4; i++){
        rgb_sendbyte(0x00);
    }

    for(uint8_t i=0; i<num; i++) {
        rgb_sendbyte(0xE0);
        rgb_sendbyte(0x00);
        rgb_sendbyte(0x00);
        rgb_sendbyte(0x00);
    }

    for(uint8_t i=0; i<4; i++){
        rgb_sendbyte(0xFF);
    }
}

/* Only for single led */
void rgb_wheel(RGB_type *led, uint8_t pos)
{
	if (pos < 85) {
		led->blue = 85 - pos;
		led->green = pos;
		led->red = 0;
		return;
	}
	if (pos < 170) {
		pos -= 85;
		led->blue = 0;
		led->green = 85 - pos;
		led->red = pos * 2;
		return;
	}
	pos -= 170;
	led->blue = pos;
	led->green = 0;
	led->red = 170 - pos * 2;
	return;
}

