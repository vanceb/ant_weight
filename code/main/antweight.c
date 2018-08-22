/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"

#include "hardware.h"
#include "sbus.h"
#include "motor.h"
#include "lights.h"

static const char *TAG = "twu";

void delay(uint32_t ms)
{
  if (ms > 0) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
  }
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
