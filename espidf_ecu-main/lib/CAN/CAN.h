#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include <CANmsg.h>
#include "can_defs.h"
#include "hardware_defs.h"
#include "packets.h"

bool CAN_start_device(bool debug_mode = false);
void Send_SOT_msg(uint8_t _msg);
mqtt_packet_t update_packet(void);
void send_can_bus_init(bluetooth data);
void send_sd_start(bluetooth data);
void send_internet_modem(bluetooth data);
void send_client_connection(bluetooth data);
void send_check_sd(bluetooth data);
bool Send_Byte_CAN(uint32_t ID, uint8_t msg);
bluetooth update_packet_sd_save(void);

/* Interrupt */
void canISR(CAN_FRAME *rxMsg);

#endif