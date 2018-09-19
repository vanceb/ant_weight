/* brushed dc motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "hardware.h"
#include "motor.h"
#include "sbus.h"
#include "lights.h"

static char TAG[] = "motor";

enum zero_centred {
    UNSIGNED = 0,
    SIGNED = 1
};


/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 * duty_cycle should be in the range 0.0 - 100.0 %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 * duty_cycle should be in the range 0.0 - 100.0 %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief motor brake
 */
static void brushed_motor_brake(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_high(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_high(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/* Normalise takes a channel value and returns a float 
 * If zero_centered is not 0 then return value is nominally in the -1.0 to 1.0 range,
 * if zero_centered is 0 then return value is nominally in the 0.0 - 1.0 range,
 * Scaling is based in the CHANNEL_MID and CHANNEL_RANGE values,
 * but limits on value are not enforced and therefore output float may be outside
 * the expected normal range.
 */
float normalise(uint16_t value, int zero_centred) 
{
    if (zero_centred)
        return ((value - CHANNEL_MID) / CHANNEL_RANGE);
    else
        return (value / (CHANNEL_RANGE * 2));
}

/* Limit modifies value so that it lies within the -1.0 to 1.0 range,
 * and provides a potential dead zone where outputs will be zero.
 * This function also enforces the failsafe.
 */
float limit(float value, float dead_zone)
{
    /* If failsafe is activated then return zero */
    if (failsafe == SBUS_RX_FAILSAFE)
        return 0.0;
    /* Make sure dead_zone is positive */
    if (dead_zone < 0.0)
        dead_zone = -dead_zone;
    /* Apply limits */
    value = value > 1.0 ? 1.0 : value;
    value = value < -1.0 ? -1.0 : value;
    value = (value > -dead_zone && value < dead_zone) ? 0.0 : value;
    return value;
}

/* Convert from channel values to a float*/
float convert(uint16_t channel_value, int zero_centered, float dead_zone)
{
    return limit(normalise(channel_value, zero_centered), dead_zone);
}

/* Calculate the left motor drive % depending on the control inputs */
float left(float speed, float turn, float max_speed, float max_turn)
{
    float val;
    val = 100.0 * (speed * max_speed + turn * max_turn);
    if(REVERSE_LEFT)
        val = -val;
    return val;
}

/* Calculate the right motor drive % depending on the control inputs */
float right(float speed, float turn, float max_speed, float max_turn)
{
    float val;
    val = 100.0 * (speed * max_speed - turn * max_turn);
    if(REVERSE_RIGHT)
        val = -val;
    return val;
}

/* Calculate the weapon motor drive % depending on the control inputs */
float weapon(float wpn, int mode)
{
    /* Weapon mode switch gives:
        SWITCH_OFF:  100% in either direction
        SWITCH_MID:  50% of demand
        SWITCH_ON:   full control
     */
    switch (mode) {
        case SWITCH_OFF:
            wpn = limit(wpn, WPN_DEAD_ZONE);
            wpn = wpn < 0.0 ? -1.0 : wpn > 0.0 ? 1.0 : 0.0;
            break;
        case SWITCH_MID:
            wpn = limit(wpn, DEAD_ZONE);
            wpn *= 0.5;
            break;
        case SWITCH_ON:
            wpn = limit(wpn, DEAD_ZONE);
            break;
        default:
            ESP_LOGE(TAG, "%s", "Weapon mode switch in unknown position");
    }

    /* Allow for reversing of weapon direction */
    if(REVERSE_WEAPON)
        wpn = -wpn;

    return 100.0 * wpn;
}

/* Drive the left motor based on an input of -1.0 - 1.0 */
void l_motor(float speed)
{
    if (speed >= 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
    } else {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
    }
}

/* Drive the right motor based on an input of -1.0 - 1.0 */
void r_motor(float speed)
{
    if (speed >= 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, speed);
    } else {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, speed);
    }
}

/* Drive the weapon motor based on an input of -1.0 - 1.0 */
void w_motor(float speed)
{
    if (speed > 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_2, speed);
    } else if (speed < 0.0) {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_2, speed);
    } else {
        /* Hold the motors where they are */
        brushed_motor_brake(MCPWM_UNIT_0, MCPWM_TIMER_2); 
    }
}

/* Initialise the pins for pwm output */
extern void motors_init(void)
{
    ESP_LOGI(TAG, "%s", "Initialising GPIO for Motors");
    /* Left Motor */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_LEFT_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_LEFT_B); 
    /* Right Motor */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_RIGHT_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_RIGHT_B); 
    /* Weapon Motor */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_WPN_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_WPN_B); 

    /* Configure the pwm peripheral */
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 40000;    //frequency = 20kHz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    /* Set up timers for each motor */
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);    
}

/* Update the motors based on the received control inputs */
extern void update_motors(uint16_t *channel)
{
    /* Get data from the SBus channels */
    /* Joysticks */
    float speed = convert(channel[0], SIGNED, DEAD_ZONE);
    float turn = convert(channel[1], SIGNED, DEAD_ZONE);
    float wpn = convert(channel[2], SIGNED, NO_DEAD_ZONE);
    //uint16_t unused = channel[3];
    /* Slide controls */
    float speed_max = convert(channel[4], UNSIGNED, NO_DEAD_ZONE);
    float turn_max = convert(channel[5], UNSIGNED, NO_DEAD_ZONE);
    /* Switches */
    int sa = switch_position(channel[6]);
    int sb = switch_position(channel[7]);
    int sc = switch_position(channel[8]);
    int sd = switch_position(channel[9]);

    /* Tweak depending on the switch positions */
    /* If inverted we want to reverse the controls */
    if (sd == SWITCH_ON)
    {
        turn = -turn;
        speed = -speed;
    }

    /* Set the mode the lights should be in */
    switch (sa) {
        case SWITCH_OFF:
            if (sc == SWITCH_OFF)
                light_mode = NORMAL;
            else
                light_mode = PIMP;
            break;
        case SWITCH_MID:
            if (sc == SWITCH_OFF)
                light_mode = HAZARD;
            else
                light_mode = ALIEN;
            break;
        case SWITCH_ON:
            if (sc == SWITCH_OFF)
                light_mode = POLICE;
            else
                light_mode = DISCO;
            break;
    }
    /* Calculate the motor drives */
    float l = left(speed, turn, speed_max, turn_max);
    float r = right(speed, turn, speed_max, turn_max);
    float w = weapon(wpn, sb);

    /* Output debigging information */
    if (sbus_rx_framecount%100==0) {
        //ESP_LOGI(TAG, "Channels: %s", outbuffer);
        ESP_LOGD(TAG, "Left: %3.0f, Right: %3.0f, Weapon: %3.0f, SA: %4d, SB: %4d, SC: %4d, SD: %4d",
                    l, r, w, sa, sb, sc, sd);
    }

    l_motor(l);
    r_motor(r);
    w_motor(w);
}

/*
void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}
*/