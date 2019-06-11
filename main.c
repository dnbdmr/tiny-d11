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
#include "samd21.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "tusb.h"
#include "rgb.h"
#include "adc.h"
#include "debug.h"
#include "htu21.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED1,	A, 17);
HAL_GPIO_PIN(D5,	A, 15);
HAL_GPIO_PIN(A5,	B, 02);

/*- Implementations ---------------------------------------------------------*/

//extern volatile uint32_t millis = 0;
volatile uint32_t millis = 0;

//-----------------------------------------------------------------------------
void irq_handler_tc3(void)
{
	if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		HAL_GPIO_LED1_toggle();
		HAL_GPIO_A5_toggle();
		TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
	}
}

void irq_handler_sys_tick(void)
{
	millis++;
}


//-----------------------------------------------------------------------------
static void timer_init(void)
{
	PM->APBCMASK.reg |= PM_APBCMASK_TC3;

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC3_GCLK_ID) |
		GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

	TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
		TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_PRESCSYNC_RESYNC;

	TC3->COUNT16.COUNT.reg = 0;

	TC3->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * 500;
	TC3->COUNT16.COUNT.reg = 0;

	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
	NVIC_EnableIRQ(TC3_IRQn);
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
	uint32_t coarse, fine;

	SYSCTRL->OSC8M.bit.PRESC = 0;

	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
		SYSCTRL_INTFLAG_DFLLRDY;

	NVMCTRL->CTRLB.bit.RWS = 2;

	coarse = NVM_READ_CAL(DFLL48M_COARSE_CAL);
	fine = NVM_READ_CAL(DFLL48M_FINE_CAL);

	SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
	while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
	SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
		SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_MODE;

	while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
		GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	SysTick_Config(48000); //systick at 1ms
}

//-----------------------------------------------------------------------------
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//-----------------------------------------------------------------------------
// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;

	// connected
	if ( dtr && rts )
	{
		// print initial message when connected
		tud_cdc_write_str("\r\nDGW-Tiny CDC device example\r\n");
	}

	//Reset into bootloader when baud is 1200 and dtr unasserted
	if (!dtr) {
		cdc_line_coding_t lc;
		tud_cdc_get_line_coding(&lc);
		if (lc.bit_rate == 1200) {
			unsigned long *a = (unsigned long *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
			*a = 0xf01669ef;
			NVIC_SystemReset();
		}
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
	tud_cdc_write_str("Stop that!\n");
	tud_cdc_read_flush();
}

void cdc_task(void)
{
	if ( tud_cdc_connected() )
	{
		/*
		// connected and there are data available
		if ( tud_cdc_available() )
		{
		uint8_t buf[64];

		// read and echo back
		uint32_t count = tud_cdc_read(buf, sizeof(buf));

		for(uint32_t i=0; i<count; i++)
		{
		tud_cdc_write_char(buf[i]);

		//if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
		}
		*/

		tud_cdc_write_flush();
	}
}
//-----------------------------------------------------------------------------
int main(void)
{
	sys_init();
	timer_init();
	rgb_init();
	tusb_init();
	debug_init();
	adc_init();
	htu21_init();

	//debug_puts("----start-----\n");

	HAL_GPIO_LED1_out();
	HAL_GPIO_LED1_set();
	HAL_GPIO_D5_out();
	HAL_GPIO_D5_clr();
	HAL_GPIO_A5_out();
	HAL_GPIO_A5_clr();

	RGB_type led;
	led.red = 0xFF;
	led.blue = 0x0;
	led.green = 0xFF;
	led.bright = 2;
	update_LEDs(&led, 1);

	int a = adc_read();
	debug_puthex(a, 8);

	char s[25];
	itoa(adc_read(), s, 10);
	debug_puts(s);
	debug_putc('\n');

	uint32_t temp;
	while (1)
	{
		if (millis > 60000) {
			HAL_GPIO_A5_toggle();
			millis = 0;
			temp = htu21_readtemp();
			itoa(temp, s, 10);
			if (tud_cdc_connected()) {
				tud_cdc_write_str("TempX100: ");
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			temp = htu21_readhumidity();
			itoa(temp, s, 10);
			if (tud_cdc_connected()) {
				tud_cdc_write_str("HumX100: ");
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			tud_cdc_write_char('\n');
			//sprintf(s, "temp: %u", (int)htu21_readtemp());
			//tud_cdc_write_str(s);
		}
		tud_task();
		cdc_task();
	}

	return 0;
}
