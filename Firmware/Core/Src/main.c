/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <extern.hpp>
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "app_touchgfx.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <File_Handling.h>
#include "Globals.hpp"
#include "sdram.h"
#include "WS2812/WS2812.hpp"
#include "SETUP/setup.hpp"
//#include "W25QXX/spi_flash.h"
//#include "MCUFlash/mcu_flash.h"

#include "FT5XX6/TargetTouch.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint8_t UART_RX_buffer[UART_RX_size];
uint8_t UART_RX_set;

uint8_t GPS_RX_buffer[GPS_RX_size];
uint8_t GPS_RX_Set;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for START_Task */
osThreadId_t START_TaskHandle;
const osThreadAttr_t START_Task_attributes = { .name = "START_Task",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for TouchGFXTask */
osThreadId_t TouchGFXTaskHandle;
const osThreadAttr_t TouchGFXTask_attributes = { .name = "TouchGFXTask",
		.stack_size = 4096 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for RGB_Task */
osThreadId_t RGB_TaskHandle;
const osThreadAttr_t RGB_Task_attributes = { .name = "RGB_Task", .stack_size =
		2048 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for LOOKUP_Task */
osThreadId_t LOOKUP_TaskHandle;
const osThreadAttr_t LOOKUP_Task_attributes = { .name = "LOOKUP_Task",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for CONFIG_Task */
osThreadId_t CONFIG_TaskHandle;
const osThreadAttr_t CONFIG_Task_attributes = { .name = "CONFIG_Task",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */
FMC_SDRAM_CommandTypeDef command;

Statuses Current_Status;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

FILE *File;

FILE *FileBuffer;
uint8_t BufferIsSet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM13_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM14_Init(void);
void Start_START_Task(void *argument);
void TouchGFX_Task(void *argument);
void Start_RGB_Task(void *argument);
void Start_LOOKUP_Task(void *argument);
void Start_CONFIG_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	BufferIsSet = 0;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_LTDC_Init();
	MX_DMA2D_Init();
	MX_FMC_Init();
	MX_CRC_Init();
	MX_TIM13_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_I2C2_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_TIM9_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM12_Init();
	MX_TIM14_Init();
	MX_TouchGFX_Init();
	/* Call PreOsInit function */
	MX_TouchGFX_PreOSInit();
	/* USER CODE BEGIN 2 */

	initAll();

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of START_Task */
	START_TaskHandle = osThreadNew(Start_START_Task, NULL,
			&START_Task_attributes);

	/* creation of TouchGFXTask */
	TouchGFXTaskHandle = osThreadNew(TouchGFX_Task, NULL,
			&TouchGFXTask_attributes);

	/* creation of RGB_Task */
	RGB_TaskHandle = osThreadNew(Start_RGB_Task, NULL, &RGB_Task_attributes);

	/* creation of LOOKUP_Task */
	LOOKUP_TaskHandle = osThreadNew(Start_LOOKUP_Task, NULL,
			&LOOKUP_Task_attributes);

	/* creation of CONFIG_Task */
	CONFIG_TaskHandle = osThreadNew(Start_CONFIG_Task, NULL,
			&CONFIG_Task_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	CAN_FilterTypeDef sFilterConfig; // declare CAN filter structure
	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	sFilterConfig.FilterBank = 15;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	// sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	__HAL_RCC_CAN1_CLK_ENABLE();
	//__HAL_RCC_CAN2_CLK_ENABLE();
	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	__HAL_RCC_CAN2_CLK_ENABLE();
	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void) {

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[0].InputAlpha = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 0) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 20;
	hltdc.Init.VerticalSync = 3;
	hltdc.Init.AccumulatedHBP = 160;
	hltdc.Init.AccumulatedVBP = 15;
	hltdc.Init.AccumulatedActiveW = 1184;
	hltdc.Init.AccumulatedActiveH = 615;
	hltdc.Init.TotalWidth = 1344;
	hltdc.Init.TotalHeigh = 635;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 1024;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 600;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg.FBStartAdress = 0xD0000000;
	pLayerCfg.ImageWidth = 1024;
	pLayerCfg.ImageHeight = 600;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	if (HAL_LTDC_DeInit(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	hltdc.Init.HorizontalSync = LCD_RES_HS - 1;
	hltdc.Init.VerticalSync = LCD_RES_VS - 1;
	hltdc.Init.AccumulatedHBP = hltdc.Init.HorizontalSync + LCD_RES_HBP; // Horizontal Synchronization Width + Horizontal Back Porch - 1
	hltdc.Init.AccumulatedVBP = hltdc.Init.VerticalSync + LCD_RES_VBP; // Vertical Synchronization Height + Vertical Back Porch - 1
	hltdc.Init.AccumulatedActiveW = hltdc.Init.AccumulatedHBP + LCD_RES_H; // Horizontal Synchronization Width + Horizontal Back Porch + Active Width - 1
	hltdc.Init.AccumulatedActiveH = hltdc.Init.AccumulatedVBP + LCD_RES_V; // Vertical Synchronization Height + Vertical Back Porch + Active Height - 1
	hltdc.Init.TotalWidth = hltdc.Init.AccumulatedActiveW + LCD_RES_HFP; //Horizontal Synchronization Width + Horizontal Back Porch + Active Width + Horizontal Front Porch - 1
	hltdc.Init.TotalHeigh = hltdc.Init.AccumulatedActiveH + LCD_RES_VFP; //Vertical Synchronization Height + Vertical Back Porch + Active Height + Vertical Front Porch - 1
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX1 = LCD_RES_H;
	pLayerCfg.WindowY1 = LCD_RES_V;
	pLayerCfg.ImageWidth = LCD_RES_H;
	pLayerCfg.ImageHeight = LCD_RES_V;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 0;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */
	HAL_TIM_MspPostInit(&htim9);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 0;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 65535;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 0;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 65535;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */
	HAL_TIM_MspPostInit(&htim11);

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void) {

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 0;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 65535;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */
	HAL_TIM_MspPostInit(&htim12);

}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void) {

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 90 - 1;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 1000 - 1;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */
	HAL_TIM_MspPostInit(&htim13);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 0;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 65535;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void) {

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = { 0 };

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 8;
	SdramTiming.SelfRefreshTime = 6;
	SdramTiming.RowCycleDelay = 7;
	SdramTiming.WriteRecoveryTime = 5;
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */
	FMC_SDRAM_CommandTypeDef command;
	if (SDRAM_Initialization_Sequence(&hsdram1, &command) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, OUT_S0_Pin | OUT_E_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOJ,
			MULTISENSE_EN2_Pin | MULTISENSE_EN3_Pin | MULTISENSE_EN4_Pin
					| MULTISENSE_EN5_Pin | MULTISENSE_EN6_Pin | LED_PJ12_Pin
					| LED_PJ13_Pin | LED_PJ14_Pin | LED_PJ15_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOK,
			MULTISENSE_EN0_Pin | MULTISENSE_EN1_Pin | MULTISENSE_RST_Pin
					| MULTISENSE_SEL0_Pin | MULTISENSE_SEL1_Pin | DET__5V_S1_Pin
					| CAN1_SEL0_Pin | CAN2_SEL0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_PI3_GPIO_Port, LED_PI3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, PUD_S0_Pin | PUD_S1_Pin | PUD_S2_Pin | PUD_E_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
			IN_E_Pin | IN_S0_Pin | IN_S1_Pin | IN_S2_Pin | IN_S3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_FLASH_GPIO_Port, SPI1_FLASH_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : BTN_1_Pin BTN_2_Pin */
	GPIO_InitStruct.Pin = BTN_1_Pin | BTN_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_EN_Pin */
	GPIO_InitStruct.Pin = TOUCH_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(TOUCH_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : HALL_OUT_1_PI12_Pin */
	GPIO_InitStruct.Pin = HALL_OUT_1_PI12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HALL_OUT_1_PI12_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OUT_S0_Pin OUT_E_Pin */
	GPIO_InitStruct.Pin = OUT_S0_Pin | OUT_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : MULTISENSE_EN2_Pin MULTISENSE_EN3_Pin MULTISENSE_EN4_Pin MULTISENSE_EN5_Pin
	 MULTISENSE_EN6_Pin LED_PJ12_Pin LED_PJ13_Pin LED_PJ14_Pin
	 LED_PJ15_Pin */
	GPIO_InitStruct.Pin = MULTISENSE_EN2_Pin | MULTISENSE_EN3_Pin
			| MULTISENSE_EN4_Pin | MULTISENSE_EN5_Pin | MULTISENSE_EN6_Pin
			| LED_PJ12_Pin | LED_PJ13_Pin | LED_PJ14_Pin | LED_PJ15_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN_3_Pin BTN_4_Pin */
	GPIO_InitStruct.Pin = BTN_3_Pin | BTN_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

	/*Configure GPIO pins : MULTISENSE_EN0_Pin MULTISENSE_EN1_Pin MULTISENSE_RST_Pin MULTISENSE_SEL0_Pin
	 MULTISENSE_SEL1_Pin DET__5V_S1_Pin */
	GPIO_InitStruct.Pin = MULTISENSE_EN0_Pin | MULTISENSE_EN1_Pin
			| MULTISENSE_RST_Pin | MULTISENSE_SEL0_Pin | MULTISENSE_SEL1_Pin
			| DET__5V_S1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

	/*Configure GPIO pin : SDIO_ENT_Pin */
	GPIO_InitStruct.Pin = SDIO_ENT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SDIO_ENT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_PI3_Pin */
	GPIO_InitStruct.Pin = LED_PI3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_PI3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PUD_S0_Pin PUD_S1_Pin PUD_S2_Pin PUD_E_Pin */
	GPIO_InitStruct.Pin = PUD_S0_Pin | PUD_S1_Pin | PUD_S2_Pin | PUD_E_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_E_Pin IN_S0_Pin IN_S1_Pin IN_S2_Pin
	 IN_S3_Pin */
	GPIO_InitStruct.Pin = IN_E_Pin | IN_S0_Pin | IN_S1_Pin | IN_S2_Pin
			| IN_S3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : CAN1_SEL0_Pin CAN2_SEL0_Pin */
	GPIO_InitStruct.Pin = CAN1_SEL0_Pin | CAN2_SEL0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_FLASH_Pin */
	GPIO_InitStruct.Pin = SPI1_FLASH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_FLASH_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float mapFloat(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long mapInt(float x, float in_min, float in_max, int out_min, int out_max) {
	return (int) ((x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == INT_PIN) {
		TOUCH_Set();
	}
}

void CheckAlerts() {

	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 0;

	if (Current_Status.RPM >= PROTECTION_RPM_HIGH) {
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
				(COLOR_RGB ) { 255, 0, 0 };
		strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
				"DANGER TO RODS");

	} else if (Current_Status.RPM >= PROTECTION_RPM_LOW) {
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text_Color = (COLOR_RGB ) {
						0, 0, 0 };
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
				(COLOR_RGB ) { 255, 255, 0 };
		strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
				"YOU SURE YOU WANT THIS");
	}

	if (Current_Status.ECT > 1200.0) {
		Current_Status.SCREEN_CONTAINERS[0].Background_Color = (COLOR_RGB ) {
						255, 0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Value.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Label.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Unit.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };

		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
				(COLOR_RGB ) { 255, 0, 0 };
		strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
				"ENGINE COOLANT °C TOO HIGH");

	} else if (Current_Status.ECT > 1050.0) {
		Current_Status.SCREEN_CONTAINERS[0].Background_Color = (COLOR_RGB ) {
						255, 255, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Value.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Label.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Unit.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };

		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
		Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
				(COLOR_RGB ) { 255, 255, 0 };
		strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
				"ENGINE COOLANT °C HIGH");
	} else {

		Current_Status.SCREEN_CONTAINERS[0].Background_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[0].Value.Text_Color = (COLOR_RGB ) {
						255, 255, 255 };
		Current_Status.SCREEN_CONTAINERS[0].Label.Text_Color = (COLOR_RGB ) {
						255, 255, 255 };
		Current_Status.SCREEN_CONTAINERS[0].Unit.Text_Color = (COLOR_RGB ) {
						255, 255, 255 };
	}

	if (Current_Status.BATT < 1100.0) {
		Current_Status.SCREEN_CONTAINERS[5].Background_Color = (COLOR_RGB ) {
						255, 0, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Value.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Label.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Unit.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
	} else if (Current_Status.BATT < 1200.0) {
		Current_Status.SCREEN_CONTAINERS[5].Background_Color = (COLOR_RGB ) {
						255, 255, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Value.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Label.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[5].Unit.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
	}

	if (Current_Status.LAMBDA1 < 900 && Current_Status.LAMBDA1 > 1100) {
		Current_Status.SCREEN_CONTAINERS[7].Background_Color = (COLOR_RGB ) {
						255, 0, 0 };
		Current_Status.SCREEN_CONTAINERS[7].Value.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[7].Label.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
		Current_Status.SCREEN_CONTAINERS[7].Unit.Text_Color = (COLOR_RGB ) { 0,
						0, 0 };
	} else {
		//Current_Status.SCREEN_CONTAINERS[4].Background_Color = (COLOR_RGB ) {255, 255, 0 };
		Current_Status.SCREEN_CONTAINERS[7].Value.Text_Color = (COLOR_RGB ) { 0,
						255, 0 };
		//Current_Status.SCREEN_CONTAINERS[4].Label.Text_Color = (COLOR_RGB ) {0, 0, 0 };
		//Current_Status.SCREEN_CONTAINERS[4].Unit.Text_Color = (COLOR_RGB ) {0, 0, 0 };
	}
}

void Update_Data() {
	for (int i = 0; i < SCREEN_CONTAINERS_COUNT; ++i) {
		switch (Current_Status.SCREEN_CONTAINERS[i].Data.Channel) {
		case CH_MGP:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.MGP;
			break;
		case CH_INJ_DC:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.INJ_DC;
			break;
		case CH_INJ_DC_ST:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.INJ_DC_ST;
			break;
		case CH_INJ_PULSE:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.INJ_PULSE;
			break;
		case CH_MAF:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.MAF;
			break;
		case CH_INJ_TIM:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.INJ_TIM;
			break;
		case CH_IGN_TIM:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.IGN_TIM;
			break;
		case CH_CAM_I_L:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.CAM_I_L;
			break;
		case CH_CAM_I_R:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.CAM_I_R;
			break;
		case CH_CAM_E_L:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.CAM_E_L;
			break;
		case CH_CAM_E_R:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.CAM_E_R;
			break;
		case CH_LAMBDA1:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.LAMBDA1;
			break;
		case CH_LAMBDA2:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.LAMBDA2;
			break;
		case CH_TRIG1_ERROR:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.TRIG1_ERROR;
			break;
		case CH_FAULT_CODES:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.FAULT_CODES;
			break;
		case CH_LF_SPEED:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.LF_SPEED;
			break;
		case CH_LR_SPEED:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.LR_SPEED;
			break;
		case CH_RF_SPEED:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.RF_SPEED;
			break;
		case CH_RR_SPEED:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.RR_SPEED;
			break;
		case CH_KNOCK1:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK1;
			break;
		case CH_KNOCK2:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK2;
			break;
		case CH_KNOCK3:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK3;
			break;
		case CH_KNOCK4:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK4;
			break;
		case CH_KNOCK5:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK5;
			break;
		case CH_KNOCK6:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK6;
			break;
		case CH_KNOCK7:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK7;
			break;
		case CH_KNOCK8:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.KNOCK8;
			break;
		case CH_LIMITS:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.LIMITS;
			break;
		case CH_TPS:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.TPS;
			break;
		case CH_ECT:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.ECT;
			break;
		case CH_IAT:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.IAT;
			break;
		case CH_ETHANOL:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.ETHANOL;
			break;
		case CH_MAP:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.MAP;
			break;
		case CH_BARO:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.BARO;
			break;
		case CH_BATT:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.BATT;
			break;
		case CH_FUELP:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.FUELP;
			break;
		case CH_OILP:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.OILP;
			break;
		case CH_FUELT:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.FUELT;
			break;
		case CH_OILT:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.OILT;
			break;
		case CH_RPM:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value = Current_Status.RPM;
			break;
		case CH_FUELL:
			Current_Status.SCREEN_CONTAINERS[i].Data.Value =
					Current_Status.FUELLEVEL;
			break;
		default:
			break;
		}
	}
	CheckAlerts();
}

void Update_RPM_Ranges() {
	Current_Status.RPM_100 = mapInt(Current_Status.RPM, 0,
	PROTECTION_RPM_HIGH, 0, 100);
	Current_Status.RPM_100 =
			Current_Status.RPM_100 >= 100 ? 100 : Current_Status.RPM_100;
	Current_Status.RPM_180 = mapInt(Current_Status.RPM, 0,
	PROTECTION_RPM_HIGH, 0, 180);
	Current_Status.RPM_180 =
			Current_Status.RPM_180 >= 180 ? 810 : Current_Status.RPM_180;
	Current_Status.RPM_270 = mapInt(Current_Status.RPM, 0,
	PROTECTION_RPM_HIGH, 0, 270);
	Current_Status.RPM_270 =
			Current_Status.RPM_270 >= 270 ? 270 : Current_Status.RPM_270;
	Current_Status.RPM_240 = mapInt(Current_Status.RPM, 0,
	PROTECTION_RPM_HIGH, 0, 240);
	Current_Status.RPM_240 =
			Current_Status.RPM_240 >= 240 ? 240 : Current_Status.RPM_240;
	Current_Status.RPM_360 = mapInt(Current_Status.RPM, 0,
	PROTECTION_RPM_HIGH, 0, 360);
	Current_Status.RPM_360 =
			Current_Status.RPM_360 >= 360 ? 360 : Current_Status.RPM_360;

	Update_Data();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

	if (hcan->Instance == CAN1) {
		Current_Status.CAN1_ACTIVE = true;
	}
	if (hcan->Instance == CAN2) {
		Current_Status.CAN2_ACTIVE = true;
	}

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)
			== HAL_OK) {
		if (Current_Status.CAN_ENABLED == 1) {

			HAL_GPIO_TogglePin(LED_PJ15_GPIO_Port, LED_PJ15_Pin);
			switch (Current_Status.CAN_PROTOCOL) {
			case CAN_LINK:
				switch (RxHeader.StdId) {
				case 0x7E8:
					switch (RxData[2]) {
					case 0x05: // PID-0x05 Engine coolant temperature , range is -40 to 215 deg C , formula == A-40
						Current_Status.ECT = RxData[3] - 40;
						break;
					case 0x0B: // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
						Current_Status.MAP = RxData[3];
						break;
					case 0x0C: // PID-0x0C , RPM  , range is 0 to 16383.75 rpm , Formula == 256A+B / 4
						Current_Status.RPM = (uint16_t) ((RxData[4] << 8)
								+ (RxData[3] & 0x00ff));
						break;
					case 0x0D: // PID-0x0D , Vehicle speed , range is 0 to 255 km/h , formula == A
						Current_Status.LF_SPEED = RxData[3];
						break;
					case 0x0E: // PID-0x0E , Ignition Timing advance, range is -64 to 63.5 BTDC , formula == A/2 - 64
						Current_Status.IGN_TIM = RxData[3] / 2 - 64;
						break;
					case 0x0F: // PID-0x0F , Inlet air temperature , range is -40 to 215 deg C, formula == A-40
						Current_Status.IAT = RxData[3] - 40;
						break;
					case 0x11: // PID-0x11 , TPS percentage, range is 0 to 100 percent, formula == 100/256 A
						Current_Status.TPS = 100 / 256 * RxData[3];
						break;
					case 0x13: // PID-0x13 , oxygen sensors present, A0-A3 == bank1 , A4-A7 == bank2
						break;
					case 0x1C: // PID-0x1C obd standard
						break;
					case 0x20: // PID-0x20 PIDs supported [21-40]
						break;
					case 0x22: // PID-0x22 Fuel /Pressure (Relative to manifold vacuum) , range is 0 to 5177.265 kPa , formula == 0.079(256A+B)
						Current_Status.FUELP = 0.079
								* (256 * (uint16_t) (RxData[4] << 8)
										+ (RxData[3] & 0x00ff));
						break;
					case 0x24: // PID-0x24 O2 sensor2, AB: fuel/air equivalence ratio, CD: voltage ,  Formula == (2/65536)(256A +B) , 8/65536(256C+D) , Range is 0 to <2 and 0 to >8V
						Current_Status.LAMBDA1 = (2 / 65536)
								* (256 * (uint16_t) (RxData[4] << 8)
										+ (RxData[3] & 0x00ff)); // , 8 / 65536 * (256 * (uint16_t)(RxData[5] << 8) + (RxData[6] & 0x00ff));
						break;
					case 0x25: // PID-0x25 O2 sensor2, AB fuel/air equivalence ratio, CD voltage ,  2/65536(256A +B) ,8/65536(256C+D) , range is 0 to <2 and 0 to >8V
						Current_Status.LAMBDA2 = (2 / 65536)
								* (256 * (uint16_t) (RxData[4] << 8)
										+ (RxData[3] & 0x00ff)); // , 8 / 65536 * (256 * (uint16_t)(RxData[5] << 8) + (RxData[6] & 0x00ff));
						break;
					case 0x33: // PID-0x33 Absolute Barometric pressure , range is 0 to 255 kPa , formula == A
						Current_Status.BARO = RxData[3];
						break;
					case 0x40: // PIDs supported [41-60]
						break;
					case 0x42: // PID-0x42 control module voltage, 256A+B / 1000 , range is 0 to 65.535v
						Current_Status.BATT = 256 * (uint16_t) (RxData[4] << 8)
								+ (RxData[3] & 0x00ff) / 1000;
						break;
					case 0x46: // PID-0x46 Ambient Air Temperature , range is -40 to 215 deg C , formula == A-40
						break;
					case 0x52: // PID-0x52 Ethanol fuel % , range is 0 to 100% , formula == (100/255)A
						Current_Status.ETHANOL = 100 / 255 * RxData[3];
						break;
					case 0x5C: // PID-0x5C Engine oil temperature , range is -40 to 210 deg C , formula == A-40
						Current_Status.OILT = RxData[3] - 40;
						break;
					case 0x60: // PIDs supported [61-80]
						break;
					default:
						break;
					}
					break;
				case 0x3E8: // Link Dash
					switch (RxData[0]) {
					case 0:
						Current_Status.RPM = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.MAP = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.MGP = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 1:
						Current_Status.BARO = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.TPS = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.INJ_DC = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 2:
						Current_Status.INJ_DC_ST = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.INJ_PULSE = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.ECT = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 3:
						Current_Status.IAT = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.BATT = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.MAF = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 4:
						Current_Status.GEAR = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.INJ_TIM = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.IGN_TIM = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 5:
						Current_Status.CAM_I_L = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.CAM_I_R = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.CAM_E_L = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 6:
						Current_Status.CAM_E_R = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.LAMBDA1 = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.LAMBDA2 = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 7:
						Current_Status.TRIG1_ERROR =
								(uint16_t) ((RxData[3] << 8)
										+ (RxData[2] & 0x00ff));
						Current_Status.FAULT_CODES =
								(uint16_t) ((RxData[5] << 8)
										+ (RxData[4] & 0x00ff));
						Current_Status.FUELP = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 8:
						Current_Status.OILT = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.OILP = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.LF_SPEED = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 9:
						Current_Status.LR_SPEED = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.RF_SPEED = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.RR_SPEED = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 10:
						Current_Status.KNOCK1 = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.KNOCK2 = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.KNOCK3 = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 11:
						Current_Status.KNOCK4 = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.KNOCK5 = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.KNOCK6 = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					case 12:
						Current_Status.KNOCK7 = (uint16_t) ((RxData[3] << 8)
								+ (RxData[2] & 0x00ff));
						Current_Status.KNOCK8 = (uint16_t) ((RxData[5] << 8)
								+ (RxData[4] & 0x00ff));
						Current_Status.LIMITS = (uint16_t) ((RxData[7] << 8)
								+ (RxData[6] & 0x00ff));
						break;
					}
					break;
				}
				break;
			case CAN_AIM:
				switch (RxHeader.StdId) {
				case 0x5F0:
					Current_Status.RPM = (uint16_t) ((RxData[1] << 8)
							+ (RxData[0] & 0x00ff));
					Current_Status.TPS = (uint16_t) ((RxData[3] << 8)
							+ (RxData[2] & 0x00ff)) / 65;
					break;
				case 0x5F2:
					Current_Status.IAT = (uint16_t) ((RxData[1] << 8)
							+ (RxData[0] & 0x00ff)) / 19 - 450;
					Current_Status.ECT = (uint16_t) ((RxData[3] << 8)
							+ (RxData[2] & 0x00ff)) / 19 - 450;
					Current_Status.FUELT = (uint16_t) ((RxData[5] << 8)
							+ (RxData[4] & 0x00ff)) / 19 - 450;
					Current_Status.OILT = (uint16_t) ((RxData[7] << 8)
							+ (RxData[6] & 0x00ff)) / 19 - 450;
					break;
				case 0x5F3:
					Current_Status.MAP = (uint16_t) ((RxData[1] << 8)
							+ (RxData[0] & 0x00ff)) / 10;
					Current_Status.BARO = (uint16_t) ((RxData[3] << 8)
							+ (RxData[2] & 0x00ff)) / 10;
					Current_Status.OILP = (uint16_t) ((RxData[5] << 8)
							+ (RxData[4] & 0x00ff)) * 100 / 100;
					Current_Status.FUELP = (uint16_t) ((RxData[7] << 8)
							+ (RxData[6] & 0x00ff)) * 100 / 2;
					break;
				case 0x5F4:
					Current_Status.BATT = (uint16_t) ((RxData[3] << 8)
							+ (RxData[2] & 0x00ff)) / 32;
					// Current_Status.GEAR = (uint16_t)((RxData[7] << 8) + (RxData[6] & 0x00ff));
					break;
				case 0x5F5:
					//Current_Status.BATT = (uint16_t)((RxData[3] << 8) + (RxData[2] & 0x00ff)) / 32;
					Current_Status.FUELLEVEL = (uint16_t) ((RxData[7] << 8)
							+ (RxData[6] & 0x00ff)) * 100 / 100;
					break;
				case 0x5F6:
					Current_Status.LAMBDA1 = (uint16_t) ((RxData[1] << 8)
							+ (RxData[0] & 0x00ff)) / 2;
					Current_Status.LAMBDA2 = (uint16_t) ((RxData[3] << 8)
							+ (RxData[2] & 0x00ff)) / 2;
					break;
				}
				break;
			default:
				break;
			}
			Update_RPM_Ranges();
		}
	}
}

void SetScreen(void) {

	//-----------------------------------------------------------------------------
	//-------------------------------LEFT------------------------------------------
	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[0].Label.Text, "ECT");
	Current_Status.SCREEN_CONTAINERS[0].Label.X = 12;
	Current_Status.SCREEN_CONTAINERS[0].Label.Y = 85;
	Current_Status.SCREEN_CONTAINERS[0].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[0].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[0].Label.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[0].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[0].Unit.Text, "°C");
	Current_Status.SCREEN_CONTAINERS[0].Unit.X = 150;
	Current_Status.SCREEN_CONTAINERS[0].Unit.Y = 85;
	Current_Status.SCREEN_CONTAINERS[0].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[0].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[0].Unit.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[0].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[0].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[0].Value.X = 12;
	Current_Status.SCREEN_CONTAINERS[0].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[0].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[0].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[0].Value.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[0].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[0].Data.Channel = CH_ECT;
	Current_Status.SCREEN_CONTAINERS[0].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[0].Data.Decimal = 0;
	Current_Status.SCREEN_CONTAINERS[0].Data.Divider = 10;
	Current_Status.SCREEN_CONTAINERS[0].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[1].Label.Text, "IAT");
	Current_Status.SCREEN_CONTAINERS[1].Label.X = 12;
	Current_Status.SCREEN_CONTAINERS[1].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[1].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[1].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[1].Label.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[1].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[1].Unit.Text, "°C");
	Current_Status.SCREEN_CONTAINERS[1].Unit.X = 100;
	Current_Status.SCREEN_CONTAINERS[1].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[1].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[1].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[1].Unit.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[1].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[1].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[1].Value.X = 12;
	Current_Status.SCREEN_CONTAINERS[1].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[1].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[1].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[1].Value.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[1].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[1].Data.Channel = CH_IAT;
	Current_Status.SCREEN_CONTAINERS[1].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[1].Data.Decimal = 0;
	Current_Status.SCREEN_CONTAINERS[1].Data.Divider = 10;
	Current_Status.SCREEN_CONTAINERS[1].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[2].Label.Text, "OIL Press");
	Current_Status.SCREEN_CONTAINERS[2].Label.X = 12;
	Current_Status.SCREEN_CONTAINERS[2].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[2].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[2].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[2].Label.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[2].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[2].Unit.Text, "kPa");
	Current_Status.SCREEN_CONTAINERS[2].Unit.X = 120;
	Current_Status.SCREEN_CONTAINERS[2].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[2].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[2].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[2].Unit.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[2].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[2].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[2].Value.X = 12;
	Current_Status.SCREEN_CONTAINERS[2].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[2].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[2].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[2].Value.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[2].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[2].Data.Channel = CH_OILP;
	Current_Status.SCREEN_CONTAINERS[2].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[2].Data.Decimal = 0;
	Current_Status.SCREEN_CONTAINERS[2].Data.Divider = 1;
	Current_Status.SCREEN_CONTAINERS[2].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[3].Label.Text, "FUEL Press");
	Current_Status.SCREEN_CONTAINERS[3].Label.X = 12;
	Current_Status.SCREEN_CONTAINERS[3].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[3].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[3].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[3].Label.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[3].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[3].Unit.Text, "kPa");
	Current_Status.SCREEN_CONTAINERS[3].Unit.X = 250;
	Current_Status.SCREEN_CONTAINERS[3].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[3].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[3].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[3].Unit.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[3].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[3].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[3].Value.X = 12;
	Current_Status.SCREEN_CONTAINERS[3].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[3].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[3].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[3].Value.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[3].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[3].Data.Channel = CH_FUELP;
	Current_Status.SCREEN_CONTAINERS[3].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[3].Data.Decimal = 0;
	Current_Status.SCREEN_CONTAINERS[3].Data.Divider = 1;
	Current_Status.SCREEN_CONTAINERS[3].Data.Default = 0;

	//-----------------------------------------------------------------------------
	//-------------------------------RIGHT-----------------------------------------
	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[4].Label.Text, "MAP");
	Current_Status.SCREEN_CONTAINERS[4].Label.X = 315;
	Current_Status.SCREEN_CONTAINERS[4].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[4].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[4].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[4].Label.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[4].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[4].Unit.Text, "kPa");
	Current_Status.SCREEN_CONTAINERS[4].Unit.X = 210;
	Current_Status.SCREEN_CONTAINERS[4].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[4].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[4].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[4].Unit.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[4].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[4].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[4].Value.X = 370;
	Current_Status.SCREEN_CONTAINERS[4].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[4].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[4].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[4].Value.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[4].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[4].Data.Channel = CH_MAP;
	Current_Status.SCREEN_CONTAINERS[4].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[4].Data.Decimal = 1;
	Current_Status.SCREEN_CONTAINERS[4].Data.Divider = 10;
	Current_Status.SCREEN_CONTAINERS[4].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[5].Label.Text, "Battery");
	Current_Status.SCREEN_CONTAINERS[5].Label.X = 315;
	Current_Status.SCREEN_CONTAINERS[5].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[5].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[5].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[5].Label.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[5].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[5].Unit.Text, "V");
	Current_Status.SCREEN_CONTAINERS[5].Unit.X = 260;
	Current_Status.SCREEN_CONTAINERS[5].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[5].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[5].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[5].Unit.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[5].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[5].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[5].Value.X = 370;
	Current_Status.SCREEN_CONTAINERS[5].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[5].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[5].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[5].Value.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[5].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[5].Data.Channel = CH_BATT;
	Current_Status.SCREEN_CONTAINERS[5].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[5].Data.Decimal = 1;
	Current_Status.SCREEN_CONTAINERS[5].Data.Divider = 100;
	Current_Status.SCREEN_CONTAINERS[5].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[6].Label.Text, "TPS");
	Current_Status.SCREEN_CONTAINERS[6].Label.X = 315;
	Current_Status.SCREEN_CONTAINERS[6].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[6].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[6].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[6].Label.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[6].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[6].Unit.Text, "%");
	Current_Status.SCREEN_CONTAINERS[6].Unit.X = 240;
	Current_Status.SCREEN_CONTAINERS[6].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[6].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[6].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[6].Unit.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[6].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[6].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[6].Value.X = 370;
	Current_Status.SCREEN_CONTAINERS[6].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[6].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[6].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[6].Value.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[6].Value.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	Current_Status.SCREEN_CONTAINERS[6].Data.Channel = CH_TPS;
	Current_Status.SCREEN_CONTAINERS[6].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[6].Data.Decimal = 0;
	Current_Status.SCREEN_CONTAINERS[6].Data.Divider = 10;
	Current_Status.SCREEN_CONTAINERS[6].Data.Default = 0;

	//-----------------------------------------------------------------------------

	strcpy(Current_Status.SCREEN_CONTAINERS[7].Label.Text, "Wideband");
	Current_Status.SCREEN_CONTAINERS[7].Label.X = 315;
	Current_Status.SCREEN_CONTAINERS[7].Label.Y = 88;
	Current_Status.SCREEN_CONTAINERS[7].Label.Width = 101;
	Current_Status.SCREEN_CONTAINERS[7].Label.Height = 30;
	Current_Status.SCREEN_CONTAINERS[7].Label.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[7].Label.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[7].Unit.Text, "Lambda");
	Current_Status.SCREEN_CONTAINERS[7].Unit.X = 115;
	Current_Status.SCREEN_CONTAINERS[7].Unit.Y = 88;
	Current_Status.SCREEN_CONTAINERS[7].Unit.Width = 101;
	Current_Status.SCREEN_CONTAINERS[7].Unit.Height = 30;
	Current_Status.SCREEN_CONTAINERS[7].Unit.Alignment = ALIGN_LEFT;
	Current_Status.SCREEN_CONTAINERS[7].Unit.Text_Color = (COLOR_RGB ) { 255,
					255, 255 };

	strcpy(Current_Status.SCREEN_CONTAINERS[7].Value.Text, "0");
	Current_Status.SCREEN_CONTAINERS[7].Value.X = 370;
	Current_Status.SCREEN_CONTAINERS[7].Value.Y = -5;
	Current_Status.SCREEN_CONTAINERS[7].Value.Width = 44;
	Current_Status.SCREEN_CONTAINERS[7].Value.Height = 96;
	Current_Status.SCREEN_CONTAINERS[7].Value.Alignment = ALIGN_RIGHT;
	Current_Status.SCREEN_CONTAINERS[7].Value.Text_Color = (COLOR_RGB ) { 255,
					0, 0 };

	Current_Status.SCREEN_CONTAINERS[7].Data.Channel = CH_LAMBDA1;
	Current_Status.SCREEN_CONTAINERS[7].Data.Adder = 0;
	Current_Status.SCREEN_CONTAINERS[7].Data.Decimal = 2;
	Current_Status.SCREEN_CONTAINERS[7].Data.Divider = 1000;
	Current_Status.SCREEN_CONTAINERS[7].Data.Default = 0;

	//-----------------------------------------------------------------------------

	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
	strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
			"Test Error Message");
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Alignment = ALIGN_CENTER;
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].X = 0;
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Y = 8;
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Width = 1024;
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Height = 80;
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
			(COLOR_RGB ) { 255, 0, 0 };
	Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text_Color = (COLOR_RGB ) { 0,
					255, 0 };

	Current_Status.SCREEN_FIELDS_CHANGED = 1;
}

void initAll(void) {

	Current_Status.LCD_BRIGHTNESS = LCD_DEFAULT_BRIGHTNESS;
	Current_Status.LCD_BRIGHTNESS_CHANGED = 1;
	htim13.Instance->CCR1 = Current_Status.LCD_BRIGHTNESS;

	Current_Status.CAN_ENABLED = 0;
	Current_Status.RGB_ENABLED = 1;
	Current_Status.RPM_SWEEP = 1;
	Current_Status.CAN_PROTOCOL = CAN_AIM;
	Current_Status.PRES_UNIT = kPa;
	Current_Status.TEMP_UNIT = C;
	Current_Status.SPEED_UNIT = Kmh;

	SetScreen();
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	initWS2812();

	HAL_GPIO_WritePin(CAN1_SEL0_GPIO_Port, CAN1_SEL0_Pin, SET);
	HAL_GPIO_WritePin(CAN2_SEL0_GPIO_Port, CAN2_SEL0_Pin, SET);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_START_Task */
/**
 * @brief  Function implementing the START_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_START_Task */
void Start_START_Task(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	osDelay(250);

	if (Current_Status.RPM_SWEEP == 1) {
		for (int i = 0; i < PROTECTION_RPM_HIGH / 100; ++i) {
			Current_Status.RPM = i * 100;
			Update_RPM_Ranges();
			osDelay(10);
		}
		for (int i = PROTECTION_RPM_HIGH / 100; i > 0; --i) {
			Current_Status.RPM = i * 100;
			Update_RPM_Ranges();
			osDelay(10);
		}
	}
	Current_Status.RPM = 0;
	Update_RPM_Ranges();

	Current_Status.CAN_ENABLED = 1;
	for (;;) {

		if (Current_Status.LCD_BRIGHTNESS_CHANGED == 1) {
			htim13.Instance->CCR1 = Current_Status.LCD_BRIGHTNESS;
			Current_Status.LCD_BRIGHTNESS_CHANGED = 0;
		}

		HAL_GPIO_TogglePin(LED_PJ12_GPIO_Port, LED_PJ12_Pin);

		Current_Status.CAN1_ACTIVE = false;
		Current_Status.CAN2_ACTIVE = false;

		osDelay(1000);
	}

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TouchGFX_Task */
/**
 * @brief Function implementing the TouchGFXTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TouchGFX_Task */
__weak void TouchGFX_Task(void *argument) {
	/* USER CODE BEGIN TouchGFX_Task */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END TouchGFX_Task */
}

/* USER CODE BEGIN Header_Start_RGB_Task */
/**
 * @brief Function implementing the RGB_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_RGB_Task */
void Start_RGB_Task(void *argument) {
	/* USER CODE BEGIN Start_RGB_Task */

	setWS2812Brightness(10);
	/* Infinite loop */
	for (;;) {
		if (Current_Status.RGB_ENABLED == 1) {
			//Current_Status.RPM = Current_Status.RPM >= PROTECTION_RPM_HIGH ? 0 : Current_Status.RPM;
			//Current_Status.RPM = Current_Status.RPM + 100;

			Current_Status.ENGINE_PROTECTION =
					Current_Status.RPM >= PROTECTION_RPM_HIGH ? 1 : 0;

			clearWS2812All();
			uint8_t RPMLED = WS2812_LED_N;

			uint16_t lowRange = mapInt(Current_Status.RPM, PROTECTION_RPM_LOW,
					0, RPMLED - PROTECTION_RPM_LED, 1);
			lowRange =
					lowRange > RPMLED - PROTECTION_RPM_LED ?
							RPMLED - PROTECTION_RPM_LED : lowRange;
			lowRange = lowRange < 1 ? 1 : lowRange;

			for (int i = 1; i <= lowRange; i++) {
				WS2812_RGB_t color;
				if (Current_Status.ENGINE_PROTECTION == 1) {
					color.red = 0;
					color.green = 255;
					color.blue = 0;
				} else {
					color.red = 0;
					color.green = 255;
					color.blue = 0;
				}
				setWS2812One((RPMLED - i) + (WS2812_LED_N - RPMLED), color);
			}

			if (Current_Status.RPM > PROTECTION_RPM_LOW) {
				uint16_t highRange = mapInt(Current_Status.RPM,
				PROTECTION_RPM_HIGH, PROTECTION_RPM_LOW,
				PROTECTION_RPM_LED, 1);
				for (int i = 1; i <= highRange; i++) {
					WS2812_RGB_t color;
					color.red = 255;
					color.green = 0;
					color.blue = 0;

					setWS2812One(
							(PROTECTION_RPM_LED - i) + (WS2812_LED_N - RPMLED),
							color);
				}

				updateWS2812();
				osDelay(50);

				for (int i = 1; i <= highRange; i++) {
					WS2812_RGB_t color;
					color.red = 0;
					color.green = 0;
					color.blue = 0;

					setWS2812One(
							(PROTECTION_RPM_LED - i) + (WS2812_LED_N - RPMLED),
							color);
				}
			}

			updateWS2812();
			osDelay(100);
		}
	}
	/* USER CODE END Start_RGB_Task */
}

/* USER CODE BEGIN Header_Start_LOOKUP_Task */
/**
 * @brief Function implementing the LOOKUP_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_LOOKUP_Task */
void Start_LOOKUP_Task(void *argument) {
	/* USER CODE BEGIN Start_LOOKUP_Task */
	/* Infinite loop */
	for (;;) {

		if (!Current_Status.CAN1_ACTIVE) {

			Current_Status.RPM = 0;
			Current_Status.RPM_RGB = 0;

			Current_Status.MGP = 0;
			Current_Status.INJ_DC = 0;
			Current_Status.INJ_DC_ST = 0;
			Current_Status.INJ_PULSE = 0;
			Current_Status.MAF = 0;
			Current_Status.INJ_TIM = 0;
			Current_Status.IGN_TIM = 0;
			Current_Status.CAM_I_L = 0;
			Current_Status.CAM_I_R = 0;
			Current_Status.CAM_E_L = 0;
			Current_Status.CAM_E_R = 0;
			Current_Status.LAMBDA1 = 0;
			Current_Status.LAMBDA2 = 0;
			Current_Status.TRIG1_ERROR = 0;
			Current_Status.FAULT_CODES = 0;
			Current_Status.LF_SPEED = 0;
			Current_Status.LR_SPEED = 0;
			Current_Status.RF_SPEED = 0;
			Current_Status.RR_SPEED = 0;
			Current_Status.KNOCK1 = 0;
			Current_Status.KNOCK2 = 0;
			Current_Status.KNOCK3 = 0;
			Current_Status.KNOCK4 = 0;
			Current_Status.KNOCK5 = 0;
			Current_Status.KNOCK6 = 0;
			Current_Status.KNOCK7 = 0;
			Current_Status.KNOCK8 = 0;
			Current_Status.LIMITS = 0;

			Current_Status.TPS = 0;
			Current_Status.ECT = 0;
			Current_Status.IAT = 0;
			Current_Status.ETHANOL = 0;
			Current_Status.MAP = 0;
			Current_Status.BARO = 0;
			Current_Status.BATT = 0;
			Current_Status.FUELP = 0;
			Current_Status.OILP = 0;
			Current_Status.FUELT = 0;
			Current_Status.OILT = 0;

			Current_Status.FUELLEVEL = 0;

			clearWS2812All();
			updateWS2812();

			Update_RPM_Ranges();
			Update_Data();

			Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled = 1;
			Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text_Color =
					(COLOR_RGB ) { 0, 0, 0 };
			Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Background_Color =
					(COLOR_RGB ) { 255, 255, 0 };
			strcpy(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Text,
					"CONNECTION LOST");

		}

		osDelay(1000);
	}
	/* USER CODE END Start_LOOKUP_Task */
}

/* USER CODE BEGIN Header_Start_CONFIG_Task */
/**
 * @brief Function implementing the CONFIG_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_CONFIG_Task */
void Start_CONFIG_Task(void *argument) {
	/* USER CODE BEGIN Start_CONFIG_Task */
	/* Infinite loop */
	for (;;) {




		//CDC_Transmit_FS(buffer, sizeof(buffer));



		if (UART_RX_set == 1) {

			uint8_t index = 0;

			switch (UART_RX_buffer[0]) {
				case 'S'://SET
					switch (UART_RX_buffer[1]) {
						case 'C'://CONTAINER
							index = ((uint8_t)UART_RX_buffer[2]) - 48;
							Set_Screen_Container(index);
							break;
					}
					break;
				case 'R'://READ
					switch (UART_RX_buffer[1]) {
						case 'V'://VERSION
							switch (UART_RX_buffer[2]) {
								case 'H'://HARDWARE
									{
										uint8_t buffer[] =  "OPFD7\r\n";
										CDC_Transmit_FS(buffer, sizeof(buffer));
									}
									break;
								case 'F'://FIRMWARE
									{
										uint8_t buffer[] =  "Version 0.1 beta\r\n";
										CDC_Transmit_FS(buffer, sizeof(buffer));
									}
									break;
							}
							break;
					}
					break;
			}
			UART_RX_set = 0;
		}
		osDelay(100);
	}
	/* USER CODE END Start_CONFIG_Task */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
