#ifndef LIGHTS_H
#define LIGHTS_H

#define BLINK_INTERVAL 500 //ms
#define UPDATE_DELAY 100 //ms - max 10Hz
#define RX_LOST_TIMEOUT 500 //ms

/* Configure the LEDs */
void ledStrandSetup(void);

/* Task to set the lights based on state */
void indicator_task(void *pvParameter);

#endif