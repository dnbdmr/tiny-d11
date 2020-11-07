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

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED1,	A, 5);
HAL_GPIO_PIN(LED2,	A, 8);
HAL_GPIO_PIN(LED3,	A, 9);

/*- Implementations ---------------------------------------------------------*/

volatile uint32_t msticks = 0;

void SysTick_Handler(void)
{
	msticks++;
}

uint32_t millis(void)
{
	uint32_t m;
	__disable_irq();
	__DMB();
	m = msticks;
	__enable_irq();
	__DMB();
	return m;
}

void delay_us(uint32_t us)
{
	if (!us || (us >= SysTick->LOAD))
		return;
	if(!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk))
		return;
	us = F_CPU/1000000*us;
	uint32_t time = SysTick->VAL;
	while ((time - SysTick->VAL) < us);
}

//-----------------------------------------------------------------------------
void TC1_Handler(void)
{
	if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		HAL_GPIO_LED1_toggle();
		HAL_GPIO_LED2_toggle();
		TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
		if (TCC0->CC[1].reg < 3000) 
			TCC0->CC[1].reg += 300;
		else
			TCC0->CC[1].reg = 0;
	}
}

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
static void sys_init(void)
{

	NVMCTRL->CTRLB.bit.RWS = 1; // Set Flash Wait States to 1 for 3.3V operation @ 48MHz

	SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
	while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)); // Wait for DFLL sync complete

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000); // Set to multiply USB SOF frequency (when USB attached)
	
	uint32_t coarse, fine;
	coarse = NVM_READ_CAL(DFLL48M_COARSE_CAL); // Read factory cals for DFLL48M
	fine = NVM_READ_CAL(DFLL48M_FINE_CAL);
	SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine); // Load factory cals

	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | 
							SYSCTRL_DFLLCTRL_USBCRM | // Set DFLL for USB Clock Recovery Mode
							SYSCTRL_DFLLCTRL_BPLCKC | // Bypass Coarse Lock, ignored with USBCRM
							SYSCTRL_DFLLCTRL_CCDIS |  // Disable Chill Cycle
							SYSCTRL_DFLLCTRL_RUNSTDBY |  // Run during standby for USB wakeup interrupts
							SYSCTRL_DFLLCTRL_MODE;   // Set Closed Loop Mode
							//SYSCTRL_DFLLCTRL_STABLE; // Fine calibration register locks (stable) after fine lock, ignored with USBCRM

	while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY)); // Wait for DFLL sync complete

	//Setup Generic Clock Generator 0 with DFLL48M as source:
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
		GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	// gclk1 at 512k
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(15635);
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC(GCLK_SOURCE_OSC8M) | GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	PM->APBCMASK.reg |= PM_APBCMASK_TCC0;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC0 | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0;
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV64_Val);
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
	TCC0->PER.reg = 3000;
	TCC0->CC[1].reg = 1500;
	while (TCC0->SYNCBUSY.reg);
	HAL_GPIO_LED3_out();
	HAL_GPIO_LED3_pmuxen(HAL_GPIO_PMUX_F);
	TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
	while (TCC0->SYNCBUSY.bit.ENABLE);
	
	SysTick_Config(48000); //systick at 1ms
}

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
//-----------------------------------------------------------------------------
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	HAL_GPIO_LED1_in();
	HAL_GPIO_LED2_in();
	HAL_GPIO_LED3_pmuxdis();
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
	HAL_GPIO_LED1_out();
	HAL_GPIO_LED2_out();
	HAL_GPIO_LED3_pmuxen(HAL_GPIO_PMUX_F);
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk); //disable systick
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

//-----------------------------------------------------------------------------
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

const char help_msg[] = \
						"Tiny usb test commands:\n" \
						"b [ms]\ttimer blink rate\n" \
						"d [0-3000]\tled pwm\n" \
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
	usb_setup();
	tusb_init();
	timer_init();
	htu21_init();
	adc_init();
	debug_init();
	dma_init();

	HAL_GPIO_LED1_out();
	HAL_GPIO_LED1_set();

	HAL_GPIO_LED3_out();

	HAL_GPIO_LED2_out();
	HAL_GPIO_LED2_clr();

	uint8_t line[25];

	while (1)
	{
		//htu21_task();
		//adc_task();
		tud_task();

		if (cdc_task(line, 25)) {
			if (line[0] == 'b') {
				uint32_t ms = atoi((const char *)&line[1]);
				if (ms > 0 && ms < 50000)
					timer_ms(ms);
			}
			else if (line[0] == 'l') {
				uint32_t dim = atoi((const char *)&line[1]);
				if (dim <= 3000)
					TCC0->CC[1].reg = dim;
			}
			else if (line[0] == 'h') {
				char s[10];
				uint32_t hum = htu21_readhumidity();
				hum /= 100;
				itoa(hum, s, 10);
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			else if (line[0] == 't') {
				char s[10];
				uint32_t temp = htu21_readtemp();
				temp /= 100;
				itoa(temp, s, 10);
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			else if (line[0] == 'o') {
				char s[10];
				uint32_t temp = adc_read()/74 - 460;
				itoa(temp, s, 10);
				tud_cdc_write_str(s);
				tud_cdc_write_char('\n');
			}
			else if (line[0] == 'c') {
				debug_putc('a');
				debug_putc('\n');
			}
			else if (line[0] == 'd') {
				dma_ch_enable(0);
			}
			else if (line[0] == 'D') {
				dma_ch_disable(0);
			}
			else if (line[0] == '?') {
				print_help();
			}
		}
	}

	return 0;
}
