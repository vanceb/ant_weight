/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <time.h>
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
//#define BLINK_GPIO_1 19
//#define BLINK_GPIO_2 18

#define BLINK_INTERVAL 500 //ms
#define UPDATE_DELAY 100 //ms - max 10Hz
#define RX_LOST_TIMEOUT 500 //ms
//#define NUM_LEDS 6

static const char *TAG = "twu";
//uint8_t leds[] = {18, 19, 22, 23, 32, 33};

void delay(uint32_t ms)
{
  if (ms > 0) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
  }
}

inline unsigned long clock_ms()
{
    return (1000 * clock()) / CLOCKS_PER_SEC;
}

// Global setup for the WS2812 LEDs
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

void flash(strand_t *strand, pixelColor_t color)
{
    int i;
    pixelColor_t dark = pixelFromRGB(0,0,0);
    for (i=0; i<strand->numPixels; i++)
        if(strand->pixels[i].r == dark.r &&
           strand->pixels[i].g == dark.g &&
           strand->pixels[i].b == dark.b
        )
            strand->pixels[i] = color;
        else
            strand->pixels[i] = dark;
}

void set_color(strand_t *strand, pixelColor_t color)
{
    int i;
    for (i=0; i<strand->numPixels; i++)
        strand->pixels[i] = color;
}

void set_front_color(strand_t *strand, pixelColor_t color)
{
    strand->pixels[0] = strand->pixels[1] = color;
}

void set_rear_color(strand_t *strand, pixelColor_t color)
{
    strand->pixels[2] = strand->pixels[3] = color;
}

void indicator_task(void *pvParameter)
{
    int i;
    unsigned long last_update = clock_ms();
    unsigned long last_good = 0;

    /* Create the pointer to the strand used by the library */
    strand_t *strand = &STRANDS[0];

    pixelColor_t dark = pixelFromRGB(0,0,0);
    pixelColor_t red = pixelFromRGB(255,0,0);
    pixelColor_t green = pixelFromRGB(0,255,0);
    pixelColor_t blue = pixelFromRGB(0,0,255);
    pixelColor_t amber = pixelFromRGB(255, 64, 0);
    pixelColor_t white = pixelFromRGB(255, 255, 255);

    for (;;) {
        switch (failsafe) {
            case SBUS_RX_OK:
                /* Normal indication */
                set_front_color(strand, white);
                set_rear_color(strand, red);
                last_update = clock_ms();
                last_good = clock_ms();
                break;
            case SBUS_RX_LOST:
                /* Reception Lost */
                /* This seems to happen a lot */
                if (last_good + RX_LOST_TIMEOUT < clock_ms()) {
                    set_color(strand, blue);
                    last_update = clock_ms();
                }
                break;
            case SBUS_RX_FAILSAFE:
                /* Failsafe initiatied */
                if ((last_update + BLINK_INTERVAL) < clock_ms()) {
                    flash(strand, amber);
                    last_update = clock_ms();
                }
                break;
            default:
                /* Should not get here! */
                ESP_LOGE(TAG, "Unknown SBus receive state");
        }
        digitalLeds_updatePixels(strand);
        delay(UPDATE_DELAY);
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
    /* Run the tasks for display LEDs and control */
    xTaskCreate(&indicator_task, "indicator_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(sbus_event_task, "sbus_event_task", 2048, NULL, 12, NULL);
}
