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

extern uint8_t UART_RX_buffer[UART_RX_size];
extern uint8_t UART_RX_set;


extern uint8_t GPS_RX_buffer[GPS_RX_size];
extern uint8_t GPS_RX_Set;


#endif /* INC_EXTERN_H_ */
