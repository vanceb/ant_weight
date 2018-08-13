#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>

#define NUM_CHANNELS 18

/* Define buffer sizes */
#define BUF_SIZE (128)
#define RD_BUF_SIZE (BUF_SIZE)

/* SBus Rx States */
enum SBus_Rx_State {
    SBUS_RX_OK,
    SBUS_RX_LOST,
    SBUS_RX_FAILSAFE
};

/* Variables that are updated from the SBus Receiver */
extern int failsafe; /* Takes one of the SBus_Rx_State values */
extern uint16_t channel[NUM_CHANNELS]; /* Decoded SBus channels */

/* Run once to initialise the uart to receive SBus */
extern int sbus_rx_init(int uart, int rx_pin);
/* Task to run to receive sbus data and decode it */
extern void sbus_event_task(void *pvParameters);

#endif