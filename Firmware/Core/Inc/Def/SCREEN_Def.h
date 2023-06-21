/*
 * IO_Def.h
 *
 *  Created on: Oct 16, 2022
 *      Author: Ben
 */

#ifndef INC_DEF_SCREEN_DEF_H_
#define INC_DEF_SCREEN_DEF_H_

typedef enum {
	LABEL = 0,
	UNIT,
	VALUE,
	BACKGROUND
} ITEM;

typedef enum {
	ALIGN_LEFT = 0,
	ALIGN_CENTER,
	ALIGN_RIGHT
} ALIGN;

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} COLOR_RGB;

typedef enum {
	CH_MGP = 0,
	CH_INJ_DC,
	CH_INJ_DC_ST,
	CH_INJ_PULSE,
	CH_MAF,
	CH_INJ_TIM,
	CH_IGN_TIM,
	CH_CAM_I_L,
	CH_CAM_I_R,
	CH_CAM_E_L,
	CH_CAM_E_R,
	CH_LAMBDA1,
	CH_LAMBDA2,
	CH_TRIG1_ERROR,
	CH_FAULT_CODES,
	CH_LF_SPEED,
	CH_LR_SPEED,
	CH_RF_SPEED,
	CH_RR_SPEED,
	CH_KNOCK1,
	CH_KNOCK2,
	CH_KNOCK3,
	CH_KNOCK4,
	CH_KNOCK5,
	CH_KNOCK6,
	CH_KNOCK7,
	CH_KNOCK8,
	CH_LIMITS,
	CH_TPS,
	CH_ECT,
	CH_IAT,
	CH_ETHANOL,
	CH_MAP,
	CH_BARO,
	CH_BATT,
	CH_FUELP,
	CH_OILP,
	CH_FUELT,
	CH_OILT,
	CH_RPM,
	CH_FUELL
} CHANNEL;

typedef struct {
	int16_t Value;
	COLOR_RGB Text_Color;
} RANGE_COLOR;

typedef struct {
	uint8_t Enabled;
	char Text[32];
	int16_t X;
	int16_t Y;
	int16_t Width;
	int16_t Height;
	ALIGN Alignment;
	COLOR_RGB Text_Color;
} FIELD;


typedef struct {
	CHANNEL Channel;
	uint16_t Value;
	uint8_t Decimal;
	uint16_t Adder;
	uint16_t Divider;
	uint16_t Default;
	RANGE_COLOR Min_Color;
	RANGE_COLOR Max_Color;
} DATA;

typedef struct {
	FIELD Value;
	FIELD Label;
	FIELD Unit;
	DATA Data;
	COLOR_RGB Background_Color;
} CONTAINER;

typedef struct {
	uint8_t Enabled;
	char Text[256];
	int16_t X;
	int16_t Y;
	int16_t Width;
	int16_t Height;
	ALIGN Alignment;
	COLOR_RGB Text_Color;
	COLOR_RGB Background_Color;
} MESSAGE_CONTAINERS;




#endif /* INC_DEF_SCREEN_DEF_H_ */
