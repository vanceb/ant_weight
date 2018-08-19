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

/* Digital switch states */
enum Switch_State {
    SWITCH_OFF,
    SWITCH_MID,
    SWITCH_ON
};

/* Define switch transitions */
#define SWITCH_OFF_VAL 200
#define SWITCH_MID_VAL 1000

/* Get switch state from value */
inline int switch_position(uint16_t value)
{
    if (value < SWITCH_OFF_VAL)
        return SWITCH_OFF;
    if (value < SWITCH_MID_VAL)
        return SWITCH_MID;
    return SWITCH_ON;
}

/* Variables that are updated from the SBus Receiver */
extern int failsafe; /* Takes one of the SBus_Rx_State values */
extern uint16_t channel[NUM_CHANNELS]; /* Decoded SBus channels */
extern unsigned long sbus_rx_framecount; /* Count he number of frames received */

/* Run once to initialise the uart to receive SBus */
extern int sbus_rx_init(int uart, int rx_pin);
/* Task to run to receive sbus data and decode it */
extern void sbus_event_task(void *pvParameters);

#endif