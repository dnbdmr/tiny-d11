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

void update_LEDs(RGB_type *leds, uint8_t num)
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

