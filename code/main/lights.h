#ifndef LIGHTS_H
#define LIGHTS_H

#define BLINK_INTERVAL 500 //ms
#define UPDATE_DELAY 40 //ms - max 25Hz
#define RX_LOST_TIMEOUT 500 //ms

enum light_modes {
    NORMAL,
    HAZARD,
    POLICE,
    DISCO,
    PIMP,
    ALIEN
};

extern int light_mode;

/* Configure the LEDs */
void ledStrandSetup(void);

/* Task to set the lights based on state */
void indicator_task(void *pvParameter);

#endif