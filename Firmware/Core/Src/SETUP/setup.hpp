/*
 * setup.hpp
 *
 *  Created on: Jun 23, 2023
 *      Author: Dev
 */

#ifndef INC_SETUP_HPP_
#define INC_SETUP_HPP_


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <extern.hpp>
#include <stdio.h>
#include <string.h>

void Set_Screen_Field(void);
void Set_Screen_Container(uint8_t index);
void Set_Screen_Message(void);
void Set_Screen_Screen(void);
void Set_Screen_Alert(void);

void Set_Container_Label(uint8_t index);
void Set_Container_Unit(uint8_t index);
void Set_Container_Value(uint8_t index);
void Set_Container_Data(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* INC_SETUP_HPP_ */
