/*
 * setup.cpp
 *
 *  Created on: Jun 23, 2023
 *      Author: Dev
 */

#include "setup.hpp"

void Set_Screen_Field(void) {

}
void Set_Screen_Container(uint8_t index) {

	////SCxL....
	switch (UART_RX_buffer[3]) {
	case 'L':
		Set_Container_Label(index);
		break;
	case 'U':
		Set_Container_Unit(index);
		break;
	case 'V':
		Set_Container_Value(index);
		break;
	case 'D':
		Set_Container_Data(index);
		break;
	}
}

void Set_Screen_Message(void) {

}
void Set_Screen_Screen(void) {

}
void Set_Screen_Alert(void) {

}

void Set_Container_Label(uint8_t index) {

	//Screen Container x LABEL
	//SCxLE:0				//ENABLED 1-True, 0 False
	//SCxLT:ECT				//TEXT
	//SCxLC:255,255,255	    //COLOUR R,G,B
	//SCxLA:0 				//ALIGHNMENT 0-LEFT, 1-RIGHT, 2-CENTER
	//SCxLX:120				//X
	//SCxLY:120				//Y
	//SCxLW:120				//WIDTH
	//SCxLH:120				//HEIGHT

	char *ptr;
	uint8_t token = 0;

	switch (UART_RX_buffer[4]) {
		case 'E':

			break;
		case 'T':
			ptr = strtok((char *)UART_RX_buffer, ":");
			while (ptr != NULL)
			{
				if(token == 1)
				{
					strcpy(Current_Status.SCREEN_CONTAINERS[index].Label.Text, ptr);
				}
				token = token + 1;
				ptr = strtok (NULL, ":");
			}
			break;
		case 'C':

			break;
		case 'X':

			break;
		case 'Y':

			break;
		case 'W':

			break;
		case 'H':

			break;
	}
}

void Set_Container_Unit(uint8_t index) {
	//Screen Container x UNIT
	//SCxUE:0				//ENABLED 1-True, 0 False
	//SCxUT:ECT				//TEXT
	//SCxUC:255,255,255	    //COLOUR R,G,B
	//SCxUA:0 				//ALIGHNMENT 0-LEFT, 1-RIGHT, 2-CENTER
	//SCxUX:120				//X
	//SCxUY:120				//Y
	//SCxUW:120				//WIDTH
	//SCxUH:120				//HEIGHT

	char *ptr;
	uint8_t token = 0;

	switch (UART_RX_buffer[4]) {
		case 'E':

			break;
		case 'T':
			ptr = strtok((char *)UART_RX_buffer, ":");
			while (ptr != NULL)
			{
				if(token == 1)
				{
					strcpy(Current_Status.SCREEN_CONTAINERS[index].Unit.Text, ptr);
				}
				token = token + 1;
				ptr = strtok (NULL, ":");
			}
			break;
		case 'C':

			break;
		case 'X':

			break;
		case 'Y':

			break;
		case 'W':

			break;
		case 'H':

			break;
	}

}

void Set_Container_Value(uint8_t index) {
	//Screen Container x VALUE
	//SCxVE:0				//ENABLED 1-True, 0 False
	//SCxVT:ECT				//TEXT
	//SCxVC:255,255,255	    //COLOUR R,G,B
	//SCxVA:0 				//ALIGHNMENT 0-LEFT, 1-RIGHT, 2-CENTER
	//SCxVX:120				//X
	//SCxVY:120				//Y
	//SCxVW:120				//WIDTH
	//SCxVH:120				//HEIGHT

	char *ptr;
	uint8_t token = 0;

	switch (UART_RX_buffer[4]) {
		case 'E':

			break;
		case 'T':
			ptr = strtok((char *)UART_RX_buffer, ":");
			while (ptr != NULL)
			{
				if(token == 1)
				{
					strcpy(Current_Status.SCREEN_CONTAINERS[index].Value.Text, ptr);
				}
				token = token + 1;
				ptr = strtok (NULL, ":");
			}
			break;
		case 'C':

			break;
		case 'X':

			break;
		case 'Y':

			break;
		case 'W':

			break;
		case 'H':

			break;
	}
}

void Set_Container_Data(uint8_t index) {

	//Screen Container x DATA
	//SCxDE:0				//ENABLED 1-True, 0 False
	//SCxDC:0				//CHANNEL
	//SCxDV:			    //VALUE
	//SCxDD:0 				//DECIMAL
	//SCxDA:-20				//ADDER
	//SCxDR:10				//DIVIDER
	//SCxDF:120				//DEFAULT VALUE
	//SCxDW0:120			//MIN
	//SCxDW1:120			//MAX
	//SCxDW0M:Message		//MIN MESSAGE
	//SCxDW1M:Message		//MAX MESSAGE
	//SCxDW0C:255,255,255	//MIN COLOR
	//SCxDW1C:255,255,255	//MAX COLOR

}

