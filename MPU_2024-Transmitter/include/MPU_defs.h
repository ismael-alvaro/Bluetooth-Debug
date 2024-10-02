#ifndef MPU_DEFS_H
#define MPU_DEFS_H

#include <Arduino.h>
#include "BajaDefs/packets.h"
#include "BajaDefs/hard_defs.h"
#include "BajaDefs/can_defs.h"

/* State Machines */
typedef enum {
    IDLE_ST, 
    RADIO_ST, 
    GPS_ST, 
    DEBUG_ST
} state_t;

//typedef enum {SENDING, LISTENING} connectivity_states;

unsigned long timer;

#endif