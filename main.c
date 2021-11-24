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
#include <string.h>
#include "tusb.h"
#include "utils.h"
#include "usb_utils.h"
#include "gpio.h"
#include "timer.h"

/*- Definitions -------------------------------------------------------------*/

/*- Implementations ---------------------------------------------------------*/
const char help_msg[] = \
						"Tiny usb test commands:\n" \
						"b [ms]\ttimer blink rate\n";

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
	gpio_init();
	timer_init();
	gpio_configure(0, GPIO_CONF_OUTPUT);

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
			case 'a':
					  cdc_write_num(tud_cdc_write_available());
					  tud_cdc_write_char('\n');
					  break;
			case '?':
				print_help();
				break;
			default:
				break;
		}
	}

	return 0;
}
