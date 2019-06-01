/*
 * Copyright (c) 2016-2017, Alex Taradov <alex@taradov.com>
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

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd21.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "debug.h"
#include "tusb.h"

/*- Definitions -------------------------------------------------------------*/
HAL_GPIO_PIN(LED1,	A, 17);
HAL_GPIO_PIN(D5,	A, 15);
HAL_GPIO_PIN(A5,	B, 02);
HAL_GPIO_PIN(USB_DM,   A, 24);
HAL_GPIO_PIN(USB_DP,   A, 25);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void irq_handler_tc3(void)
{
  if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
  {
    HAL_GPIO_LED1_toggle();
    HAL_GPIO_D5_toggle();
	HAL_GPIO_A5_toggle();
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

  HAL_GPIO_USB_DM_pmuxen(PORT_PMUX_PMUXE_G_Val);
  HAL_GPIO_USB_DP_pmuxen(PORT_PMUX_PMUXE_G_Val);
}
//-----------------------------------------------------------------------------
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
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
    tud_cdc_write_str("\r\nTinyUSB CDC MSC HID device example\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

void cdc_task(void)
{
  if ( tud_cdc_connected() )
  {
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

      tud_cdc_write_flush();
    }
  }
}
//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  timer_init();
  debug_init();

  debug_puts("\r\n--- start ---\r\n");

  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) |
      GCLK_CLKCTRL_GEN(0);
  tusb_init();

  HAL_GPIO_LED1_out();
  HAL_GPIO_LED1_set();
  HAL_GPIO_D5_out();
  HAL_GPIO_D5_clr();
  HAL_GPIO_A5_out();
  HAL_GPIO_A5_clr();

  while (1)
  {
	  tud_task();
	  cdc_task();
  }

  return 0;
}

