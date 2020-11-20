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

void rtc_init(void)
{
	//PM->APBAMASK.reg |= PM_APBAMASK_RTC;

	/*
	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(NVM_READ_CAL(OSC32K_CAL)) | SYSCTRL_OSC32K_RUNSTDBY |
		SYSCTRL_OSC32K_EN1K | SYSCTRL_OSC32K_ENABLE;
	while (!SYSCTRL->PCLKSR.bit.OSC32KRDY);
	*/

	//GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_GENEN;
	//GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(2) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_RTC;

	RTC->MODE2.CTRL.reg = RTC_MODE2_CTRL_SWRST;
	while (RTC->MODE2.CTRL.bit.SWRST);
	while (RTC->MODE2.CTRL.bit.ENABLE);

	RTC->MODE2.CTRL.reg = RTC_MODE2_CTRL_MODE_CLOCK | RTC_MODE2_CTRL_ENABLE;
	while (RTC->MODE2.STATUS.bit.SYNCBUSY);
}
