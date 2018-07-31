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
    //ESP_LOGD(TAG, "Starting blink task");
}

void app_main()
{
    ESP_LOGD(TAG, "Minimal Stack Size: %d", configMINIMAL_STACK_SIZE);
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
