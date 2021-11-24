#include <stdbool.h>
#include "gpio.h"
#include "sam.h"
#include "timer.h"


void timer_ms(uint32_t ms) {
	TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 1024) * ms;
	TC1->COUNT16.COUNT.reg = 0;
}

void timer_init(void)
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
void TC1_Handler(void)
{
	if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
		gpio_toggle(0);
	}
}

