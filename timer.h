#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

/* Initalize timer and interrupt */
void timer_init(void);

/* Set period of timer in ms */
void timer_ms(uint32_t ms);

#endif
