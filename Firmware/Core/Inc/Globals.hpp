#ifndef __GLOBALS_H
#define __GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include <Def/CAN_Def.h>
#include <Def/UNIT_Def.h>
#include <Def/IO_Def.h>
#include <Def/STORAGE_Def.hpp>
#include <Def/SCREEN_Def.h>
#include <Def/SCREEN_Def.h>

#define PSI_TO_BAR 				0.0689476f
#define PSI_TO_KPA 				6.89476f
#define BAR_TO_PSI 				14.5038f
#define BAR_TO_KPA 				100.0f
#define KPA_TO_BAR 				0.01f
#define KPA_TO_PSI 				0.145038f
#define AFR_TO_LAMBDA 			0.06802721088f

#define SCREEN_CONTAINERS_COUNT 8

#define UART_RX_size 			2048
#define GPS_RX_size 			2048
#define CAN_SETUP_ID			0x580

#define OPF_HID_EPIN_SIZE		0x40U
#define OPF_HID_EPOUT_SIZE		0x40U


//#define USE_1280x480
#define USE_1024x600

#if defined(USE_1280x480)
	#define LCD_RES_H  480
	#define LCD_RES_HS  2
	#define LCD_RES_HBP  2
	#define LCD_RES_HFP  2
	#define LCD_RES_V  1280
	#define LCD_RES_VS  20
	#define LCD_RES_VBP  40
	#define LCD_RES_VFP  10
#elif defined(USE_1024x600)
	#define LCD_RES_H  1024
	#define LCD_RES_HS  20
	#define LCD_RES_HBP  140
	#define LCD_RES_HFP  160
	#define LCD_RES_V  600
	#define LCD_RES_VS  3
	#define LCD_RES_VBP  12
	#define LCD_RES_VFP  20
#endif



typedef enum {
	CAN_LINK = 0,
	CAN_AIM,
	CAN_MX5
} CANDefEnum;

typedef struct {
	CANDefEnum CAN_PROTOCOL;

	uint8_t CAN1_ACTIVE;
	uint8_t CAN2_ACTIVE;
	uint8_t CAN_ENABLED;
	uint8_t RGB_ENABLED;
	uint8_t RPM_SWEEP;
	uint8_t ACTIVE_SCREEN;

	UnitDefEnum PRES_UNIT;
	UnitDefEnum TEMP_UNIT;
	UnitDefEnum SPEED_UNIT;

	uint16_t LED_BRIGHTNESS;
	uint8_t LED_BRIGHTNESS_CHANGED;

	uint16_t LCD_BRIGHTNESS;
	uint8_t LCD_BRIGHTNESS_CHANGED;

	//BTN
	uint8_t BTN_TOP_RIGHT;
	uint8_t BTN_TOP_LEFT;
	uint8_t BTN_BOTTOM_RIGHT;
	uint8_t BTN_BOTTOM_LEFT;

	CONTAINER SCREEN_CONTAINERS[SCREEN_CONTAINERS_COUNT];
	MESSAGE_CONTAINERS SCREEN_MESSAGE_CONTAINERS[1];

	uint8_t SCREEN_FIELDS_CHANGED;

	uint16_t PROTECTION_RPM_LOW;
	uint16_t PROTECTION_RPM_HIGH;
	uint16_t PROTECTION_RPM_LED;
	uint16_t PROTECTION_OIL_LOW;
	uint16_t PROTECTION_FUEL_LOW;

}Settings;

typedef struct {

	uint16_t RPM;
	uint16_t RPM_100;
	uint16_t RPM_180;
	uint16_t RPM_240;
	uint16_t RPM_270;
	uint16_t RPM_360;

	uint16_t MGP;
	uint16_t INJ_DC;
	uint16_t INJ_DC_ST;
	uint16_t INJ_PULSE;
	uint16_t MAF;
	uint16_t INJ_TIM;
	uint16_t IGN_TIM;
	uint16_t CAM_I_L;
	uint16_t CAM_I_R;
	uint16_t CAM_E_L;
	uint16_t CAM_E_R;
	uint16_t LAMBDA1;
	uint16_t LAMBDA2;
	uint16_t TRIG1_ERROR;
	uint16_t FAULT_CODES;
	uint16_t LF_SPEED;
	uint16_t LR_SPEED;
	uint16_t RF_SPEED;
	uint16_t RR_SPEED;
	uint16_t KNOCK1;
	uint16_t KNOCK2;
	uint16_t KNOCK3;
	uint16_t KNOCK4;
	uint16_t KNOCK5;
	uint16_t KNOCK6;
	uint16_t KNOCK7;
	uint16_t KNOCK8;
	uint16_t LIMITS;

	uint16_t TPS;
	int16_t ECT;
	int16_t IAT;
	uint16_t ETHANOL;
	int16_t MAP;
	uint16_t BARO;
	uint16_t BATT;
	uint16_t FUELP;
	uint16_t OILP;
	int16_t FUELT;
	int16_t OILT;

	int16_t FUELLEVEL;

	uint16_t GPS_LATITUDE;
	uint16_t GPS_LONGITUDE;
	uint16_t GPS_FIXED;
	uint16_t GPS_SATTELITES;
	uint16_t GPS_ALTITUDE;
	uint16_t GPS_SPEED;

	uint16_t GEAR;
	uint8_t ENGINE_PROTECTION;

	uint16_t IND_LEFT;
	uint16_t IND_HIGH;
	uint16_t IND_FUEL;
	uint16_t IND_OIL;
	uint16_t IND_BATT;
	uint16_t IND_PARK;
	uint16_t IND_DTC;
	uint16_t IND_ECT;
	uint16_t IND_LOW;
	uint16_t IND_RIGHT;

} Statuses;

#ifdef __cplusplus
}
#endif

#endif // GLOBALS_H
