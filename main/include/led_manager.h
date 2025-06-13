#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <stdint.h>


typedef enum {
    LED_STATE_OFF,
    LED_STATE_RED,
    LED_STATE_GREEN,
    LED_STATE_BLUE
} led_state_t;


// LED_RGB_WS2812
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_command_t;

void led_init(void);
void led_set_color(uint8_t red, uint8_t green, uint8_t blue);
void led_blink(uint8_t red, uint8_t green, uint8_t blue, int delay_ms, int times);



#endif /* LED_MANAGER_H */