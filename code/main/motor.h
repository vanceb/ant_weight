#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Value below which motors do not drive */
#define NO_DEAD_ZONE 0.0
#define DEAD_ZONE 0.05
#define WPN_DEAD_ZONE 0.8

/* Allow for motor direction changes */
#define REVERSE_LEFT 0
#define REVERSE_RIGHT 0
#define REVERSE_WEAPON 1

/* What is the paramerters for a servo channel? */
#define CHANNEL_MID 992
#define CHANNEL_MIN 172
#define CHANNEL_MAX 1811
#define CHANNEL_RANGE ((CHANNEL_MAX - CHANNEL_MIN) / 2.0)

/* Set up pwm */
/* Uses pin definitions from hardware.h */
extern void motors_init(void);

/* High level control of motors from SBus channels */
extern void update_motors(uint16_t *channel);

#endif