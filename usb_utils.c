/*
 * Copyright (c) 2021, DNBDMR <dnbdmr@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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

#include "sam.h"
#include "tusb.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "gpio.h"
#include "utils.h"

/*- Definitions -------------------------------------------------------------*/

/*- Implementations ---------------------------------------------------------*/
void usb_setup(void)
{
	// Enable USB Clocks
	PM->APBBMASK.reg |= PM_APBBMASK_USB;
	PM->AHBMASK.reg |= PM_AHBMASK_USB;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) |
		GCLK_CLKCTRL_GEN(0);

	// Enable USB pins
	PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
	PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));
}

void USB_Handler(void)
{
	dcd_int_handler(0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	gpio_configure(0, GPIO_CONF_INPUT);
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk); //disable systick
	uint32_t *a = (uint32_t *)(0x40000838); // Disable BOD12, SAMD11 errata #15513
	*a = 0x00000004;
	//SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
	*a = 0x00000006; // Enable BOD12, SAMD11 errata #15513
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	gpio_configure(0, GPIO_CONF_OUTPUT);
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk); //disable systick
	SysTick_Config(48000); //systick at 1ms
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;

	// connected
	if ( dtr && rts )
	{
		// print initial message when connected
		//tud_cdc_write_str("Hello!\n");
	}

	//Reset into bootloader when baud is 1200 and dtr unasserted
	if (!dtr) {
		cdc_line_coding_t lc;
		tud_cdc_get_line_coding(&lc);
		if (lc.bit_rate == 1200) {
			NVIC_SystemReset();
		}
	}
}

// Invoked when CDC interface received data from host
/*
void tud_cdc_rx_cb(uint8_t itf)
{
	(void) itf;
	//tud_cdc_write_str("Stop That!!\n");
	//tud_cdc_read_flush();
}
*/

/* Retrieves full line from cdc, returns true when found */
uint8_t cdc_task(uint8_t line[], uint8_t max)
{
	static uint8_t pos = 0;
	uint8_t success = 0;

	if (tud_cdc_connected() && tud_cdc_available()) {	// connected and there are data available
		uint8_t buf[64];
		uint8_t count = tud_cdc_read(buf, sizeof(buf));

		for (uint8_t i=0; i<count; i++) {
			tud_cdc_write_char(buf[i]);
			if (pos < max-1) {
				if ((line[pos] = buf[i]) == '\n') {
					success = 1;
				}
				pos++;
			}
		}
	}

	tud_cdc_write_flush(); // Freeze without this

	if (success) {
		success = 0;
		line[pos] = '\0';
		pos = 0;
		return 1;
	}
	else
		return 0;
}

void cdc_write_num(int32_t num)
{
	char s[20];
	itoa2(num, s);
	tud_cdc_write_str(s);
}

