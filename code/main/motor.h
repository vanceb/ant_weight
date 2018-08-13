#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Define IO Pins for PWM drive */
#define GPIO_LEFT_A 19
#define GPIO_LEFT_B 18
#define GPIO_RIGHT_A 33
#define GPIO_RIGHT_B 32
#define GPIO_WPN_A 23
#define GPIO_WPN_B 22

/* What proportion should we mix in the turning to the speed? */
#define TURN_MIX 1.0
/* Value below which motors do not drive */
#define DEAD_ZONE 5.0

/* What is the paramerters for a servo channel? */
#define CHANNEL_MID 992
#define CHANNEL_MIN 172
#define CHANNEL_MAX 1811
#define CHANNEL_RANGE ((CHANNEL_MAX - CHANNEL_MIN) / 2.0)

/* Set up pwm */
extern void motors_init(void);

/* Functions to work out how much to drive each motor */
extern float left(uint16_t fwd, uint16_t turn);
extern float right(uint16_t fwd, uint16_t turn);

/* Functions to drive each motor */
extern void l_motor(float speed);
extern void r_motor(float speed);
extern void w_motor(float speed);

#endif