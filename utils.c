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

#include <stdint.h>
#include <sam.h>
#include "nvm_data.h"

// System ---------------------------------------------------------------------
void sys_init(void)
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

	SysTick_Config(48000); //systick at 1ms
}

// Systick --------------------------------------------------------------------
static volatile uint32_t msticks = 0;

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

// Utils ----------------------------------------------------------------------
int atoi2(const char *str)
{
	if (*str == '\0')
		return 0;

	int res = 0;  // Initialize result
	int sign = 1;  // Initialize sign as positive
	int i = 0;   // Initialize index of first digit

	while (str[i] == ' ' || str[i] == '\t' || str[i] == '\n' || str[i] == '\v')
		i++; // Skip whitespace

	if (str[i] == '-') {
		sign = -1;
		i++;
	}

	for (; str[i] != '\0'; ++i)	{
		if (str[i] < '0' || str[i] > '9') // If string contain character it will terminate
			break; 
		res = res*10 + str[i] - '0';
	}

	return sign*res;
}

/* from: https://gist.github.com/madex/c5cd5c6a23965a845d6e */
static const uint32_t subtractors[] = {1000000000, 100000000, 10000000, 1000000,
		100000, 10000, 1000, 100, 10, 1};
char *itoa2(int32_t num, char *string) {
	char n, *str = string;
	const uint32_t *sub = subtractors;
	uint32_t u = num < 0 ? (uint32_t) -num : (uint32_t) num;    
	uint8_t  i = 10;
	while (i > 1 && u < *sub) {
		i--;
		sub++;
	}
	if (num < 0) {
		*str++ = '-';
	}
	while (i--) {
		n = '0';
		while (u >= *sub) {
			u -= *sub;
			n++;
		}
		*str++ = n;
		sub++;
	}
	*str = 0;
	return string;
}
