/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp32_digital_led_lib.h"
#include "sbus.h"
#include "motor.h"

#define SBUS_RX_PIN  16
#define BLINK_GPIO_1 19
#define BLINK_GPIO_2 18

#define NUM_LEDS 6
static const char *TAG = "blink";
uint8_t leds[] = {18, 19, 22, 23, 32, 33};

void delay(uint32_t ms)
{
  if (ms > 0) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
  }
}

void blink_task(void *pvParameter)
{
    int i;
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    //ESP_LOGD(TAG, "Starting blink task");
    for (i=0; i<NUM_LEDS; i++) {
        ESP_LOGD(TAG, "Configuring gpio %d", leds[i]);
        gpio_pad_select_gpio(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
    }
    while(1) {
        for (i=0; i<NUM_LEDS; i++) {
        ESP_LOGD(TAG, "Setting gpio %d", leds[i]);
            gpio_set_level(leds[i], 1);
            delay(500);
        }
        for (i=0; i<NUM_LEDS; i++) {
        ESP_LOGD(TAG, "Resetting gpio %d", leds[i]);
            gpio_set_level(leds[i], 0);
            delay(500);
        }
    }
    vTaskDelete(NULL);
}


// Global setup for the LEDs
strand_t STRANDS[] = {
    {
        .rmtChannel = 1,
        .gpioNum = 12,
        .ledType = LED_WS2812B_V3,
        .brightLimit = 32,
        .numPixels = 4,
        .pixels = NULL,
        ._stateVars = NULL
    },
};

int STRANDCNT = sizeof(STRANDS)/sizeof(STRANDS[0]);

/* Convenience function to set up the LED strands
*/
void ledStrandSetup(void) {
    int i;
    // GPIO setup for each strand defined above 
    for(i=0; i<STRANDCNT; i++) {
        // Convert ints into correct types
        gpio_num_t gpioNumNative = (gpio_num_t)(STRANDS[i].gpioNum);
        // Enable GPIO for the pins from IOMUX
        gpio_pad_select_gpio(gpioNumNative);
        // Set direction and initial value
        gpio_set_direction(gpioNumNative, GPIO_MODE_OUTPUT);
        gpio_set_level(gpioNumNative, 1);
        // No need to set pullup as this is an output!!
    }

    // Library-specific setup
    if(digitalLeds_initStrands(STRANDS, STRANDCNT)) {
        ESP_LOGE(TAG, "LED Strand Init Failure:  Halting...");
        while(true) {};
    }
    ESP_LOGD(TAG, "Initialised strands");
}


void indicator_task(void *pvParameter)
{
    int i;
    /* Create the pointer to the strand used by the library */
    strand_t *strand = &STRANDS[0];

    for (;;) {
        for (i=0; i<strand->numPixels; i++) {
            strand->pixels[i] = pixelFromRGB(32,16,0);   
        }
        digitalLeds_updatePixels(strand);
        delay(500);
        for (i=0; i<strand->numPixels; i++) {
            strand->pixels[i] = pixelFromRGB(0,0,0);   
        }
        digitalLeds_updatePixels(strand);
        delay(500);
    }
    vTaskDelete(NULL);
}


void app_main()
{
    /* Set up the SBus Receiver */
    sbus_rx_init(UART_NUM_1, SBUS_RX_PIN);

    /* Set up the LED Neopixels */
    ledStrandSetup();

    /* Set up PWM for motors */
    motors_init();

    ESP_LOGD(TAG, "Minimal Stack Size: %d", configMINIMAL_STACK_SIZE);
    //xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&indicator_task, "indicator_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(sbus_event_task, "sbus_event_task", 2048, NULL, 12, NULL);
}
