#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"

#include "sbus.h"
#include "motor.h"

/* Define logging tag for the module */
static const char TAG[] = "sbusRx";

/* Global variable to hold the configured UART number */
static int EX_UART_NUM;

/* Initialise global extern variables listed in the header */
int failsafe = SBUS_RX_FAILSAFE;
uint16_t channel[NUM_CHANNELS];

/* Create a queue for the UART events */
static QueueHandle_t uart0_queue;

/* uart - should be one of the UARTs UART_NUM_0, 1, or 2 */
int sbus_rx_init(int uart, int rx_pin)
{
    int resp;
    /* Store the UART number for future use */
    EX_UART_NUM = uart;
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    if ((resp = uart_param_config(EX_UART_NUM, &uart_config)) != ESP_OK)
        return resp;
    //Set UART pins (Only changing Rx pin)
    if (
        (resp = uart_set_pin(EX_UART_NUM,
                             UART_PIN_NO_CHANGE, 
                             rx_pin, 
                             UART_PIN_NO_CHANGE, 
                             UART_PIN_NO_CHANGE)
        )
        != ESP_OK)
            return resp;
    // Invert the received data
    if ((resp = uart_set_line_inverse(EX_UART_NUM, UART_INVERSE_RXD)) != ESP_OK)
        return resp;
    //Install UART driver, and get the queue.
    if (
        (resp = uart_driver_install(EX_UART_NUM,
                            BUF_SIZE * 2,
                            BUF_SIZE * 2,
                            20,
                            &uart0_queue,
                            0)
        )
        != ESP_OK)
            return resp;
    return ESP_OK;
}

/* This function must be called with a 25 byte array of sbus data
 * and there should be 18 channels for the decoded data storage.
 *
 * Reverse engineering the signal and base code from:
 * http://forum.arduino.cc/index.php/topic,99708.0.html
 * http://www.robotmaker.eu/ROBOTmaker/quadcopter-3d-proximity-sensing/sbus-graphical-representation
 */ 
static int sbus_decode(uint8_t *sbus, uint16_t *channel) 
{
    /* Check to see that we have the framing bytes */
    if (sbus[0] != 0xF || sbus[24] != 0x0)
        /* Return error if not */
        return 0;

    /* Decode the channels */
    /* 11 bit "analog" channels (0-15) */
    channel[0]  = ((sbus[1]     | sbus[2]<<8)  & 0x07FF);
    channel[1]  = ((sbus[2]>>3  | sbus[3]<<5)  & 0x07FF);
    channel[2]  = ((sbus[3]>>6  | sbus[4]<<2 | sbus[5]<<10)   & 0x07FF);
    channel[3]  = ((sbus[5]>>1  | sbus[6]<<7)  & 0x07FF);
    channel[4]  = ((sbus[6]>>4  | sbus[7]<<4)  & 0x07FF);
    channel[5]  = ((sbus[7]>>7  | sbus[8]<<1 | sbus[9]<<9)    & 0x07FF);
    channel[6]  = ((sbus[9]>>2  | sbus[10]<<6) & 0x07FF);
    channel[7]  = ((sbus[10]>>5 | sbus[11]<<3) & 0x07FF);
    channel[8]  = ((sbus[12]    | sbus[13]<<8) & 0x07FF);
    channel[9]  = ((sbus[13]>>3 | sbus[14]<<5) & 0x07FF);
    channel[10] = ((sbus[14]>>6 | sbus[15]<<2 | sbus[16]<<10) & 0x07FF);
    channel[11] = ((sbus[16]>>1 | sbus[17]<<7) & 0x07FF);
    channel[12] = ((sbus[17]>>4 | sbus[18]<<4) & 0x07FF);
    channel[13] = ((sbus[18]>>7 | sbus[19]<<1 | sbus[20]<<9)  & 0x07FF);
    channel[14] = ((sbus[20]>>2 | sbus[21]<<6) & 0x07FF);
    channel[15] = ((sbus[21]>>5 | sbus[22]<<3) & 0x07FF);
    /* Digital channels (16-17) */
    channel[16] = ((sbus[23])      & 0x0001) ? 1 : 0;
    channel[17] = ((sbus[23] >> 1) & 0x0001) ? 0 : 0;
    /* Failsafe */
    failsafe = ((sbus[23] >> 2) & 0x0001) ? SBUS_RX_LOST : SBUS_RX_OK;
    failsafe = ((sbus[23] >> 3) & 0x0001) ? SBUS_RX_FAILSAFE : failsafe;
        //if ((sbus[23] >> 2) & 0x0001) lost++;  
    return 1;
}

void sbus_event_task(void *pvParameters)
{
    int i, j;
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint16_t channel[NUM_CHANNELS];
    for(j=0;;++j) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            char outbuffer[130];
            bzero(dtmp, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    if (event.size == 25) {
                        if(sbus_decode(dtmp, channel)) {
                            if (j%50==0) {
                                for(i=0; i<NUM_CHANNELS; i++) {
                                    sprintf(&outbuffer[i*6], "%4d  ", channel[i]);
                                }
                                //ESP_LOGI(TAG, "Channels: %s", outbuffer);
                                ESP_LOGD(TAG, "Left: %3.0f, Right: %3.0f, Weapon: %3.0f",
                                         left(channel[0], channel[1], channel[4], channel[5]),
                                         right(channel[0], channel[1], channel[4], channel[5]),
                                         weapon(channel[2])
                                        );
                            }
                            update_motors(channel);
                        } else {
                            ESP_LOGE(TAG, "%s", "Decode of SBus Data failed");
                        }
                    } else {
                        /* Just display the raw SBus data */
                        /* Reduce the amount of output to the terminal */
                        if (j%50==0) {
                            for(i=0; i<event.size && i<32; i++)
                                sprintf(&outbuffer[i*4], "%3d ", dtmp[i]);
                            ESP_LOGD(TAG, "SBUS: %d bytes: %s", event.size, outbuffer);
                        }
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}