/* APA102/DotStar bitbang library for onboard dotstar on ItsyBitsy M0 express */
#ifndef _RGB_H_
#define _RGB_H_

#include <stdint.h>

typedef struct {
    uint8_t bright;
    uint8_t blue;
    uint8_t green;
    uint8_t red;
} RGB_type;

void rgb_sendbyte(uint8_t byte);
void rgb_update(RGB_type *leds, uint8_t num);
void rgb_init(void);
void rgb_zero(uint8_t num);
void rgb_wheel(RGB_type *led, uint8_t pos);

#endif // _RGB_H_ 
