/*
 * extern.h
 *
 *  Created on: May 14, 2020
 *      Author: Dev
 */

#ifndef __EXTERN_H_
#define __EXTERN_H_

#include "Globals.hpp"

extern Statuses Current_Status;
extern Settings Dash_Settings;

extern uint8_t USB_TX_Buffer[OPF_HID_EPIN_SIZE];
extern uint8_t USB_RX_Buffer[OPF_HID_EPOUT_SIZE];
extern uint8_t USB_RX_Ready;

extern uint8_t UART_RX_buffer[UART_RX_size];
extern uint8_t UART_RX_Ready;

extern uint8_t GPS_RX_buffer[GPS_RX_size];
extern uint8_t GPS_RX_Ready;


#endif /* INC_EXTERN_H_ */
