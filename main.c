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
#include "htu21.h"
#include "rgb.h"
#include "adc.h"
#include "adafruit_ptc.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED1,	A, 17);
HAL_GPIO_PIN(A5,	B, 2);
HAL_GPIO_PIN(D5,	A, 15);

RGB_type led;

/*- Implementations ---------------------------------------------------------*/

volatile uint32_t millis = 0;
void irq_handler_sys_tick(void)
{
       millis++;
}


//-----------------------------------------------------------------------------
void irq_handler_tc3(void)
{
	if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		HAL_GPIO_LED1_toggle();
		TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
	}
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

	TC3->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * 500; // 500ms
	TC3->COUNT16.COUNT.reg = 0;

	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
	NVIC_EnableIRQ(TC3_IRQn);
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{

	SYSCTRL->OSC8M.bit.PRESC = 0; // Set OSC8M prescaler to 1

	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY |  // Clear SYSCTL interrupt flags, necessary?
							SYSCTRL_INTFLAG_BOD33DET |
							SYSCTRL_INTFLAG_DFLLRDY;

	NVMCTRL->CTRLB.bit.RWS = 1; // Set Flash Wait States to 1 for 3.3V operation

	uint32_t coarse, fine;
	coarse = NVM_READ_CAL(DFLL48M_COARSE_CAL); // Read factory cals for DFLL48M
	fine = NVM_READ_CAL(DFLL48M_FINE_CAL);

	SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
	while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)); // Wait for DFLL sync complete

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000); // Set to multiply USB SOF frequency (when USB attached)
	SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine); // Load factory cals

	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | 
							SYSCTRL_DFLLCTRL_USBCRM | // Set DFLL for USB Clock Recovery Mode
							SYSCTRL_DFLLCTRL_BPLCKC | // Bypass Coarse Lock, ignored with USBCRM
							SYSCTRL_DFLLCTRL_CCDIS |  // Disable Chill Cycle
							SYSCTRL_DFLLCTRL_RUNSTDBY |  // Run during standby for USB wakeup interrupts
							SYSCTRL_DFLLCTRL_MODE;   // Set Closed Loop Mode
							//SYSCTRL_DFLLCTRL_STABLE; // Fine calibration register locks (stable) after fine lock, ignored with USBCRM

	while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)); // Wait for DFLL sync complete

	//Setup Generic Clock Generator 0 with DFLL48M as source:
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
		GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	SYSCTRL->OSC8M.bit.PRESC = 0x01; // Set OSC8M prescaler to 2
	//Setup Generic Clock Generator 3 with OSC8M as source:
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_SRC(GCLK_SOURCE_OSC8M) |
		GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;

	SysTick_Config(48000); //systick at 1ms
}

void usb_setup(void)
{
  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) |
      GCLK_CLKCTRL_GEN(0);

  // Set up USB pins
  PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
  PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
  PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
  PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
  PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
  PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));
}

//-----------------------------------------------------------------------------
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	rgb_zero(1);
	HAL_GPIO_LED1_in();
	HAL_GPIO_A5_in();
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk); //disable systick
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk << SCB_SCR_SLEEPDEEP_Pos;
	__WFI();
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	rgb_update(&led, 1);
	HAL_GPIO_LED1_out();
	HAL_GPIO_A5_out();
	SysTick_Config(48000); //systick at 1ms
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
	}

	//Reset into bootloader when baud is 1200 and dtr unasserted
	if (!dtr) {
		cdc_line_coding_t lc;
		tud_cdc_get_line_coding(&lc);
		if (lc.bit_rate == 1200) {
			unsigned long *a = (unsigned long *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4); // Make a boot key at end of RAM
			*a = 0xf01669ef; // Set boot key to uf2 magic value
			NVIC_SystemReset();
		}
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
	HAL_GPIO_A5_toggle();
	//tud_cdc_write_str("Stop That!!\n");
	//tud_cdc_read_flush();

	if ( tud_cdc_connected() )
	{
		// connected and there are data available
		if ( tud_cdc_available() )
		{
			uint8_t buf[64];
			char s[5]; 
			// read and echo back
			uint8_t count = tud_cdc_read(buf, sizeof(buf));

			for(uint32_t i=0; i<count; i++)
			{
				tud_cdc_write_char(buf[i]);
				/*
				tud_cdc_write_str("char: ");
				itoa(buf[i], s, 10);
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
				*/

				//if ( buf[i] == '\r' ) tud_cdc_write_char('\n');
			}
		}
	}
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
		}
		*/

		//tud_cdc_read_flush(); //Won't read with this
		tud_cdc_write_flush(); // Freeze without this
	}
}
//-----------------------------------------------------------------------------
int main(void)
{
	sys_init();
	usb_setup();
	tusb_init();
	timer_init();
	htu21_init();
	rgb_init();
	adc_init();

	HAL_GPIO_LED1_out();
	HAL_GPIO_LED1_out();
	HAL_GPIO_A5_out();
	HAL_GPIO_A5_set();
	HAL_GPIO_D5_out();

	led.red = 0xFF;
	led.blue = 0x0;
	led.green = 0xFF;
	led.bright = 0x02;
	rgb_update(&led, 1);

	struct adafruit_ptc_config touchA4;
	adafruit_ptc_get_config_default(&touchA4);
	touchA4.pin = 5;
	touchA4.yline = 3;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(PTC_GCLK_ID) | 	\
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(3);
	PM->APBCMASK.reg |= PM_APBCMASK_PTC;
	adafruit_ptc_init(PTC, &touchA4);
	touchA4.threshold = adafruit_ptc_single_conversion(&touchA4) + 100;

	char s[25];
	uint32_t temp;
	uint32_t minutetick = millis;
	uint32_t tenthmintick = millis;
	uint8_t ledpos = 0;

	while (1)
	{
		if ((millis - tenthmintick) >= 100) {
			tenthmintick = millis;
			temp = adafruit_ptc_single_conversion(&touchA4);
			if ( temp  > touchA4.threshold)
				HAL_GPIO_D5_set();
			else
				HAL_GPIO_D5_clr();

			rgb_wheel(&led, ledpos);
			rgb_update(&led, 1);
			ledpos += 4;
		}

		if ((millis - minutetick) >= 60000) {
			minutetick = millis;
			temp = htu21_readtemp();
			itoa(temp, s, 10);
			if (tud_cdc_connected()) {
				tud_cdc_write_str("TempX100: ");
				tud_cdc_write_str(s);
				tud_cdc_write_char('\t');
			}
			temp = htu21_readhumidity();
			itoa(temp, s, 10);
			if (tud_cdc_connected()) {
				tud_cdc_write_str("HumX100: ");
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			temp = adc_read();
			itoa(temp, s, 10);
			if (tud_cdc_connected()) {
				tud_cdc_write_str("ADC: ");
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			//led.green = led.green ? 0x00 : 0xF0;
			//rgb_update(&led, 1);
		}

		tud_task();
		cdc_task();
	}

	return 0;
}
