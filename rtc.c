/*
 * Copyright (c) 2020, DNBDMR <alex@taradov.com>
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

#include <rtc.h>
#include "nvm_data.h"
#include  "sam.h"

// debug
#include "hal_gpio.h"
HAL_GPIO_PIN(CLK, A, 16)
// HAL_GPIO_PIN(TICK, A, 15)

void rtc_init(void)
{
	// RTC
	SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_ENABLE |
							SYSCTRL_XOSC32K_RUNSTDBY |
							SYSCTRL_XOSC32K_XTALEN |
							SYSCTRL_XOSC32K_AAMPEN |
							SYSCTRL_XOSC32K_EN32K;
	SYSCTRL->XOSC32K.bit.ONDEMAND = 0;
	while (!SYSCTRL->PCLKSR.bit.XOSC32KRDY);

	/* GENDIV doesn't seem to have an effect above 6 when
	 *  DIVSEL is set
	 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(14);
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
						GCLK_GENCTRL_SRC_XOSC32K |
						GCLK_GENCTRL_OE |
						GCLK_GENCTRL_IDC |
						GCLK_GENCTRL_DIVSEL |
						GCLK_GENCTRL_GENEN;
	while (GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN(2) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_RTC;

	// HAL_GPIO_CLK_out();
	// HAL_GPIO_CLK_pmuxen(HAL_GPIO_PMUX_H);

	PM->APBAMASK.reg |= PM_APBAMASK_RTC;

	RTC->MODE2.CTRL.reg = RTC_MODE2_CTRL_MODE_CLOCK | RTC_MODE2_CTRL_PRESCALER_DIV512 | RTC_MODE2_CTRL_ENABLE;
	RTC->MODE2.READREQ.reg |= RTC_READREQ_RREQ | RTC_READREQ_RCONT;
	RTC->MODE2.FREQCORR.reg |= RTC_FREQCORR_VALUE(120) | RTC_FREQCORR_SIGN;
	while (RTC->MODE0.STATUS.bit.SYNCBUSY);
}
