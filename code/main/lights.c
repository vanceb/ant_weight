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
int light_mode = NORMAL;
unsigned long last_update;

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

void flash(strand_t *strand, pixelColor_t color, unsigned long interval)
{
    int i;
    if(last_update + interval < clock_ms()) {
        pixelColor_t dark = pixelFromRGB(0,0,0);
        for (i=0; i<strand->numPixels; i++)
            if(strand->pixels[i].r == dark.r &&
            strand->pixels[i].g == dark.g &&
            strand->pixels[i].b == dark.b
            )
                strand->pixels[i] = color;
            else
                strand->pixels[i] = dark;
        last_update = clock_ms();
    }
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

void lights_normal(strand_t *strand)
{
    pixelColor_t white = pixelFromRGB(255, 255, 255);
    pixelColor_t red = pixelFromRGB(255, 0, 0);
    set_front_color(strand, white);
    set_rear_color(strand, red);
    last_update = clock_ms();
}

void lights_police(strand_t *strand)
{
    const int seq_length = 4;
    static int pattern[] = {0, 1, 2, 1};
    static int sequence = 0;
    pixelColor_t colors[] = {pixelFromRGB(0,0,0), pixelFromRGB(0,0,255), pixelFromRGB(255,0,0)};
    int i;
    for(i=0; i<strand->numPixels; i++) {
       strand->pixels[i] = colors[pattern[(sequence + i) % seq_length]];
    }
    sequence++;
    last_update = clock_ms();
}

void lights_disco(strand_t *strand)
{
    int i;
    uint8_t r, g, b;
    for (i=0; i<strand->numPixels; i++) {
        r = 1 << ((uint8_t)esp_random() & 0x07);
        g = 1 << ((uint8_t)esp_random() & 0x07);
        b = 1 << ((uint8_t)esp_random() & 0x07);
        strand->pixels[i] = pixelFromRGB(r,g,b);
    }   
    last_update = clock_ms();
}

void lights_pimp(strand_t *strand)
{
    static int sequence = 0;
    static int update = 1;
    sequence += update;
    pixelColor_t pink = pixelFromRGB(1 << sequence, 0, 1 << sequence);   
    set_color(strand, pink);
    /* Update ready for next time */
    if (sequence == 7 || sequence ==0)
        update = -update;
    last_update = clock_ms();
}

void lights_alien(strand_t *strand)
{
    int i;
    static int led = 0;
    pixelColor_t dark = pixelFromRGB(0,0,0);
    pixelColor_t green = pixelFromRGB(0,255,0);
    for (i=0; i<strand->numPixels; i++)
    {
        if(i == led) {
            strand->pixels[i] = green;
        } else {
            strand->pixels[i] = dark;
        }
    }
    led = (led + 1) % strand->numPixels;
    last_update = clock_ms();
}

void indicator_task(void *pvParameter)
{
    last_update = clock_ms();
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
                switch (light_mode) {
                    case NORMAL:
                        lights_normal(strand);
                        break;
                    case HAZARD:
                        flash(strand, amber, BLINK_INTERVAL);
                        break;
                    case POLICE:
                        lights_police(strand);
                        break;
                    case PIMP:
                        lights_pimp(strand);
                        break;
                    case ALIEN:
                        lights_alien(strand);
                        break;
                    case DISCO:
                        lights_disco(strand);
                        break;
                    default:
                        ESP_LOGE(TAG, "%s", "Unknown light sequence");
                }
                /* Update timers */
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
                set_color(strand,red);
                last_update = clock_ms();
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
