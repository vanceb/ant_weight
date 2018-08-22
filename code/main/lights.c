#include "lights.h"
#include "hardware.h"
#include "sbus.h"
#include "esp32_digital_led_lib.h"

#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "lights";

/* Get milliseconds since the last restart */
inline unsigned long clock_ms()
{
    return (1000 * clock()) / CLOCKS_PER_SEC;
}

/* Global setup for the WS2812 LEDs */
strand_t STRANDS[] = {
    {
        .rmtChannel = 1,
        .gpioNum = LEDS_PIN,
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
        vTaskDelay(UPDATE_DELAY / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
