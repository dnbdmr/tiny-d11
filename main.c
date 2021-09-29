/*
 * Copyright (c) 2016-2017, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "sam.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "tusb.h"
#include "adc.h"
#include "htu21.h"
#include "debug.h"
#include "dma.h"
#include "rtc.h"
#include "pwm.h"
#include "utils.h"
#include "usb_utils.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED1,	A, 4)	// Timer ISR
HAL_GPIO_PIN(LED2,	A, 27)	// Timer ISR

/*- Implementations ---------------------------------------------------------*/
void TC1_Handler(void)
{
	static bool pwmdir = false;
	static int pwm = 0;
	if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
		HAL_GPIO_LED1_toggle();
		HAL_GPIO_LED2_toggle();
		if (pwm >= 223)
			pwmdir = false;
		if (pwm <= 47)
			pwmdir = true;
		if (pwmdir)
			pwm += 16;
		else
			pwm -= 16;
		pwm_write(1, pwm);
	}
}

static void timer_ms(uint32_t ms) {
	TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * ms;
	TC1->COUNT16.COUNT.reg = 0;
}

static void timer_init(void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_TC1;

	//GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID) |
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC1_TC2 |
		GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

	TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
		TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_PRESCSYNC_RESYNC;

	TC1->COUNT16.COUNT.reg = 0;

	TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * 500; // 500ms
	TC1->COUNT16.COUNT.reg = 0;

	TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
	NVIC_EnableIRQ(TC1_IRQn);
}

//-----------------------------------------------------------------------------
void adc_task(void) 
{
	static uint32_t time = 0;
	if (((millis() - time) > 10000) && tud_cdc_connected()) {
		time = millis();
		int32_t a = calculate_temperature(adc_read());
		a = (a * 9 / 5) + 32000;
		char s[15];
		itoa(a, s, 10);
		tud_cdc_write_str("CPU Tempx1000=");
		tud_cdc_write_str(s);
		tud_cdc_write_char('\n');
	}
}

void htu21_task(void)
{
	static uint32_t time = 0;
	if (((millis() - time) > 10000) && tud_cdc_connected()) {
		time = millis();
		char s[25];
		uint32_t temp;

		temp = htu21_readtemp();
		itoa(temp, s, 10);
		tud_cdc_write_str("TempX100: ");
		tud_cdc_write_str(s);
		tud_cdc_write_char('\t');

		temp = htu21_readhumidity();
		itoa(temp, s, 10);
		tud_cdc_write_str("HumX100: ");
		tud_cdc_write_str(s);
		tud_cdc_write_char('\t');
	}
}

//-----------------------------------------------------------------------------
const char help_msg[] = \
						"Tiny usb test commands:\n" \
						"b [ms]\ttimer blink rate\n" \
						"d [0-255]\tled pwm\n" \
						"h\tprint humidity\n" \
						"t\tprint local temp\n" \
						"o\tprint remote temp\n" \
						"c\tprint to debug uart\n" \
						"d\tDMA uart enable\n" \
						"D\tDMA uart disable\n";

void print_help(void)
{
	size_t len = strlen(help_msg);
	size_t pos = 0;
	while (pos < len) {
		uint32_t avail = tud_cdc_write_available();
		if ((len - pos) > avail) {
		   tud_cdc_write(&help_msg[pos], avail);
		} else {
			tud_cdc_write(&help_msg[pos], len - pos);
		}	
		pos += avail;
		tud_task();
	}
}

//-----------------------------------------------------------------------------
int main(void)
{
	sys_init();
	rtc_init(); // DEBUG: before usb incase of stall

	usb_setup();
	tusb_init();
	timer_init();
	pwm_init(TCC_CTRLA_PRESCALER_DIV1024_Val, 255);

	HAL_GPIO_LED1_out();

	HAL_GPIO_LED2_out();
	HAL_GPIO_LED2_clr();

	uint8_t line[25];

	while (1)
	{
		tud_task();

		if (!cdc_task(line, 25))
			continue;
		switch (line[0]) {
			case 'b': {
				uint32_t ms = atoi2((char *)&line[1]);
				if (ms > 0 && ms < 50000)
					timer_ms(ms);
				break;
					  }
			case 'l': {
				uint32_t dim = atoi2((char *)&line[1]);
				if (dim <= 3000)
					TCC0->CC[1].reg = dim;
				break;
					  }
			case 'd':
				dma_ch_enable(0);
				break;
			case 'D':
				dma_ch_disable(0);
				break;
			case '?':
				print_help();
				break;
			case 'S': {
				uint32_t time;
				time = atoi2((char *)&line[2]);
				RTC->MODE2.CLOCK.reg = time;
				break;
					  }
			case 's':
				cdc_write_num(RTC->MODE2.CLOCK.bit.SECOND);
				while (RTC->MODE0.STATUS.bit.SYNCBUSY);
				tud_cdc_write_char('\n');
				break;
			case 'm':
				cdc_write_num(RTC->MODE2.CLOCK.bit.MINUTE);
				tud_cdc_write_char('\n');
				break;
			case 'h':
				cdc_write_num(RTC->MODE2.CLOCK.bit.HOUR);
				tud_cdc_write_char('\n');
				break;
			case 't':
				cdc_write_num(RTC->MODE2.CLOCK.bit.HOUR);
				tud_cdc_write_char(':');
				cdc_write_num(RTC->MODE2.CLOCK.bit.MINUTE);
				tud_cdc_write_char(':');
				cdc_write_num(RTC->MODE2.CLOCK.bit.SECOND);
				tud_cdc_write_char('\n');
				break;
			case 'T':
				// Print raw RTC CLOCK register
				cdc_write_num(RTC->MODE2.CLOCK.reg);
				tud_cdc_write_char('\n');
				break;
			case 'c':
				cdc_write_num(rtc_getCorrection());
				tud_cdc_write_char('\n');
				break;
			case 'C': {
				int8_t corr = atoi2((char *)&line[2]);
				rtc_setCorrection(corr);
				break;
					  }
			default:
				break;
		}
	}

	return 0;
}
