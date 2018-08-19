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

#include "motor.h"
#include "sbus.h"

static char TAG[] = "motor";

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
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

/* Normalise takes a channel value and returns a float */
float normalise(uint16_t value, int zero_centred) 
{
    if (zero_centred)
        return ((value - CHANNEL_MID) / CHANNEL_RANGE);
    else
        return (value / (CHANNEL_RANGE * 2));
}

/* Limit modifies value so that it lies within a range and with a potential dead zone */
float limit(float value, float dead_zone)
{
    /* Make sure dead_zone is positive */
    if (dead_zone < 0.0)
        dead_zone = -dead_zone;
    /* Apply limits */
    value = value > 1.0 ? 1.0 : value;
    value = value < -1.0 ? -1.0 : value;
    value = (value > -dead_zone && value < dead_zone) ? 0.0 : value;
    /* If failsafe is activated then zero all outputs */
    value = (failsafe == SBUS_RX_FAILSAFE) ? 0.0 : value;
    return value;
}

extern float left(uint16_t fwd, uint16_t turn, uint16_t max_speed, uint16_t max_turn)
{
    return 100.0 * limit(((limit(normalise(fwd, 1), DEAD_ZONE) * normalise(max_speed, 0)) + 
                             (limit(normalise(turn, 1), DEAD_ZONE) * normalise(max_turn, 0))), 0.0);
}
extern float right(uint16_t fwd, uint16_t turn, uint16_t max_speed, uint16_t max_turn)
{
    return 100.0 * limit(((limit(normalise(fwd, 1), DEAD_ZONE) * normalise(max_speed, 0)) -
                             (limit(normalise(turn, 1), DEAD_ZONE) * normalise(max_turn, 0))), 0.0);
}
extern float weapon(uint16_t wpn)
{
    return 100.0 * limit(normalise(wpn, 1), WPN_DEAD_ZONE);
}

extern void l_motor(float speed)
{
    if (speed >= 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
    } else {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
    }
}
extern void r_motor(float speed)
{
    if (speed >= 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_1, speed);
    } else {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_1, speed);
    }
}
extern void w_motor(float speed)
{
    if (speed >= 0.0) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_2, speed);
    } else {
        speed = -speed;
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_2, speed);
    }
}
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

extern void update_motors(uint16_t *channel)
{
    uint16_t speed = channel[0];
    uint16_t turn = channel[1];
    uint16_t wpn = channel[2];
    uint16_t speed_max = channel[4];
    uint16_t turn_max = channel[5];
    

    l_motor(left(speed, turn, speed_max, turn_max));
    r_motor(right(speed, turn, speed_max, turn_max));
    w_motor(weapon(wpn));
}

/*
void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}
*/