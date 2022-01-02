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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "usbd_cdc_if.h"
#include "task.h"
#include <stdio.h>
#include <stdbool.h>

// Sensors
#include "BME280/bme280_defs.h"
#include "BME280/bme280.h"

// SD
#include "File_Handling_RTOS.h"


#include "LCD/spi_ili9341.h"




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct 							//Queue
{
	char Buf[1024];
}QUEUE_t;

volatile unsigned long ulHighFreqebcyTimerTicks;		// This variable using for calculate how many time all tasks was running.
char str_management_memory_str[1000] = {0};
int freemem = 0;

uint32_t tim_val = 0;

// For SD works (use in fatfs_sd.c file)
volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

// LCD
extern uint16_t TFT9341_WIDTH;
extern uint16_t TFT9341_HEIGHT;
// LCD DMA
uint8_t dma_spi_fl=0;
uint32_t dma_spi_cnt=1;
//

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Blue_LED_Blink */
osThreadId_t Blue_LED_BlinkHandle;
uint32_t Blue_LED_BlinkBuffer[ 1024 ];
osStaticThreadDef_t Blue_LED_BlinkControlBlock;
const osThreadAttr_t Blue_LED_Blink_attributes = {
  .name = "Blue_LED_Blink",
  .cb_mem = &Blue_LED_BlinkControlBlock,
  .cb_size = sizeof(Blue_LED_BlinkControlBlock),
  .stack_mem = &Blue_LED_BlinkBuffer[0],
  .stack_size = sizeof(Blue_LED_BlinkBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Show_Resources */
osThreadId_t Show_ResourcesHandle;
uint32_t Show_ResourcesBuffer[ 500 ];
osStaticThreadDef_t Show_ResourcesControlBlock;
const osThreadAttr_t Show_Resources_attributes = {
  .name = "Show_Resources",
  .cb_mem = &Show_ResourcesControlBlock,
  .cb_size = sizeof(Show_ResourcesControlBlock),
  .stack_mem = &Show_ResourcesBuffer[0],
  .stack_size = sizeof(Show_ResourcesBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
uint32_t UART_TaskBuffer[ 500 ];
osStaticThreadDef_t UART_TaskControlBlock;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .cb_mem = &UART_TaskControlBlock,
  .cb_size = sizeof(UART_TaskControlBlock),
  .stack_mem = &UART_TaskBuffer[0],
  .stack_size = sizeof(UART_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for bme280 */
osThreadId_t bme280Handle;
uint32_t bme280Buffer[ 500 ];
osStaticThreadDef_t bme280ControlBlock;
const osThreadAttr_t bme280_attributes = {
  .name = "bme280",
  .cb_mem = &bme280ControlBlock,
  .cb_size = sizeof(bme280ControlBlock),
  .stack_mem = &bme280Buffer[0],
  .stack_size = sizeof(bme280Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AM2302 */
osThreadId_t AM2302Handle;
uint32_t AM2302Buffer[ 500 ];
osStaticThreadDef_t AM2302ControlBlock;
const osThreadAttr_t AM2302_attributes = {
  .name = "AM2302",
  .cb_mem = &AM2302ControlBlock,
  .cb_size = sizeof(AM2302ControlBlock),
  .stack_mem = &AM2302Buffer[0],
  .stack_size = sizeof(AM2302Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SD_CARD */
osThreadId_t SD_CARDHandle;
uint32_t SD_CARDBuffer[ 500 ];
osStaticThreadDef_t SD_CARDControlBlock;
const osThreadAttr_t SD_CARD_attributes = {
  .name = "SD_CARD",
  .cb_mem = &SD_CARDControlBlock,
  .cb_size = sizeof(SD_CARDControlBlock),
  .stack_mem = &SD_CARDBuffer[0],
  .stack_size = sizeof(SD_CARDBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
uint32_t LCDBuffer[ 11000 ];
osStaticThreadDef_t LCDControlBlock;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .cb_mem = &LCDControlBlock,
  .cb_size = sizeof(LCDControlBlock),
  .stack_mem = &LCDBuffer[0],
  .stack_size = sizeof(LCDBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTQueue */
osMessageQueueId_t UARTQueueHandle;
uint8_t UARTQueueBuffer[ 10 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t UARTQueueControlBlock;
const osMessageQueueAttr_t UARTQueue_attributes = {
  .name = "UARTQueue",
  .cb_mem = &UARTQueueControlBlock,
  .cb_size = sizeof(UARTQueueControlBlock),
  .mq_mem = &UARTQueueBuffer,
  .mq_size = sizeof(UARTQueueBuffer)
};
/* USER CODE BEGIN PV */



// BME280 part /////////////////////////////////////////////////////////////////////////////////////
struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

int8_t init_bme280(void);
void bme280_measure(void);

//----------------------------------------------------------------------------------------
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c3, (id << 1), &reg_addr, 1, 10) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c3, (id << 1) | 0x01, data, len, 10) != HAL_OK) return -1;

  return 0;
}
//----------------------------------------------------------------------------------------
void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}
//----------------------------------------------------------------------------------------
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);

  if(HAL_I2C_Master_Transmit(&hi2c3, (id << 1), (uint8_t*)buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;

  free(buf);
  return 0;
}
// End BME280 part/////////////////////////////////////////////////////////////////////////////////////

//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//{
//	uint32_t test_micros = SystemCoreClock;
//	micros *= (SystemCoreClock / 100000) /168;			// 84
//	while (micros--);
//	//HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
//
//}


bool delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	tim_val = us/10;
	HAL_TIM_Base_Start_IT(&htim10);
	while(tim_val != 0)
	{

	}
	HAL_TIM_Base_Stop_IT(&htim10);
	tim_val = 0;
	int s = 99;
	return true;

//	__HAL_TIM_SET_COUNTER(&htim10, 0);
//	HAL_TIM_Base_Start_IT(&htim10);
//	uint32_t kk = __HAL_TIM_GET_COUNTER(&htim10);
//	while (__HAL_TIM_GET_COUNTER(&htim10) < us)  // wait for the counter to reach the us input in the parameter
//	{
//
//	}
//	//HAL_TIM_Base_Stop_IT(&htim10);
//	return true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	 if(hspi == &hspi2)
	  {
	    dma_spi_cnt--;
	    if(dma_spi_cnt==0)
	    {
	      HAL_SPI_DMAStop(&hspi2);
	      dma_spi_cnt=1;
	      dma_spi_fl=1;
	    }
	  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);
void Start_Blue_LED_Blink(void *argument);
void Start_Show_Resources(void *argument);
void Start_UART_Task(void *argument);
void Start_bme280(void *argument);
void Start_AM2302(void *argument);
void Start_SD_CARD(void *argument);
void Start_LCD(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_RTC_Init();

  MX_DMA_Init();
  MX_SPI2_Init();

  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);		//  This TIM3 using for calculate how many time all tasks was running.

  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim10);			// Using for generate us delays
  HAL_TIM_Base_Start_IT(&htim1);			// Blink Green LED




//  osDelay(1000);
//	  ILI9341_Draw_Text( "TEST 1234567890 !!!", 5,0, WHITE, 2, BLACK);
//
////	  ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 150, 150, RED);
////	  ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 100, 100, BLUE);
////	  ILI9341_Draw_Filled_Rectangle_Coord(50, 50, 200, 200, WHITE);
////	  ILI9341_Draw_Filled_Rectangle_Coord(30, 30, 200, 200, YELLOW);


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

  /* Create the queue(s) */
  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (10, sizeof(QUEUE_t), &UARTQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Blue_LED_Blink */
  Blue_LED_BlinkHandle = osThreadNew(Start_Blue_LED_Blink, NULL, &Blue_LED_Blink_attributes);

  /* creation of Show_Resources */
  Show_ResourcesHandle = osThreadNew(Start_Show_Resources, NULL, &Show_Resources_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(Start_UART_Task, NULL, &UART_Task_attributes);

  /* creation of bme280 */
  bme280Handle = osThreadNew(Start_bme280, NULL, &bme280_attributes);

  /* creation of AM2302 */
  AM2302Handle = osThreadNew(Start_AM2302, NULL, &AM2302_attributes);

  /* creation of SD_CARD */
  SD_CARDHandle = osThreadNew(Start_SD_CARD, NULL, &SD_CARD_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(Start_LCD, NULL, &LCD_attributes);

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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x23;
  sTime.Minutes = 0x59;
  sTime.Seconds = 0x45;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x28;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x10;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, T_MOSI_Pin|CS_I2C_SPI_Pin|CS_LCD_Pin|RESET_LCD_Pin
                          |DC_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AM2302_Pin|CS_microSD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, T_CLK_Pin|T_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : T_MOSI_Pin CS_I2C_SPI_Pin CS_LCD_Pin */
  GPIO_InitStruct.Pin = T_MOSI_Pin|CS_I2C_SPI_Pin|CS_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_LCD_Pin DC_LCD_Pin */
  GPIO_InitStruct.Pin = RESET_LCD_Pin|DC_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin AM2302_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|AM2302_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_microSD_Pin */
  GPIO_InitStruct.Pin = CS_microSD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_microSD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : T_CLK_Pin T_CS_Pin */
  GPIO_InitStruct.Pin = T_CLK_Pin|T_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T_MISO_Pin */
  GPIO_InitStruct.Pin = T_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//    LCD_init();

//  LCD_init();

  for(;;)
  {
//	  osDelay(1000);
//	  ILI9341_Draw_Text( "TEST 1234567890 !!!", 5,0, WHITE, 2, BLACK);
	  osDelay(10);
	 // speed_test_LCD(10);
//	  ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 150, 150, RED);
//	  ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 100, 100, BLUE);
//	  ILI9341_Draw_Filled_Rectangle_Coord(50, 50, 200, 200, WHITE);
//	  ILI9341_Draw_Filled_Rectangle_Coord(30, 30, 200, 200, YELLOW);


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Blue_LED_Blink */
/**
* @brief Function implementing the Blue_LED_Blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Blue_LED_Blink */
void Start_Blue_LED_Blink(void *argument)
{
  /* USER CODE BEGIN Start_Blue_LED_Blink */
  /* Infinite loop */

	// Task every seconds blink blue LED and send data in virtual com port.
	// Set up RTC
	/*	README <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	 * STM32IDE  automatically generated setup code and fill in RTC structure fields seconds, minutes,
	 * hours and date. So, for avoid rewriting time, you need comment automated generated code.
	 *
	 * RTC needs BATTERY for count time when external power will be removed.
	 * For STM32F407 discovery dev board needs remove R26, and connect battery to VBAT (near R26).
	 * Also, need solder the LF Crystal and two capacitors.
	 */

	// 1. Set time
	  RTC_TimeTypeDef sTime = {0};
//	  sTime.Hours = 9;
//	  sTime.Minutes = 33;
//	  sTime.Seconds = 00;
//	  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  // Set date

	  RTC_DateTypeDef sDate = {0};
//	  sDate.Date = 29;
//	  sDate.Month = RTC_MONTH_DECEMBER;
//	  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
//	  sDate.Year = 21;
//	  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  /////////////////////////////////////////////////////////////////////

	QUEUE_t msg;												// Make a queue

	char buff[50] = {0};
	char buf[5] = {0};
	char str_end_of_line[4] = {'\r','\n','\0'};

	//LCD_init();
	//lcd_test_print();

	static uint8_t i = 1;
	for(;;)
	{
//		speed_test_LCD(5);

		// Blue LED blink
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
		osDelay(900);

		// RTC part
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);						// Get time (write in sDime struct)
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);						// Get data (write in sDime struct)

		memset(msg.Buf, 0, sizeof(msg.Buf));								// Fill in buff '\0'
		memset(buff, 0, sizeof(buff));

		strcat(msg.Buf, "RTC DATA AND TIME >>>>>>>    " );

		// Date
		itoa(sDate.Year, buf, 10);
		strcat(msg.Buf, buf);

		itoa(sDate.Month, buf, 10);
		strcat(msg.Buf, "-");
		strcat(msg.Buf, buf);

		itoa(sDate.Date, buf, 10);
		strcat(msg.Buf, "-");
		strcat(msg.Buf, buf);

		strcat(msg.Buf, " | ");

		// Time
		itoa(sTime.Hours, buf, 10);
		strcat(msg.Buf, buf);

		itoa(sTime.Minutes, buf, 10);
		strcat(msg.Buf, ":");
		strcat(msg.Buf, buf);

		itoa(sTime.Seconds, buf, 10);
		strcat(msg.Buf, ":");
		strcat(msg.Buf, buf);

		strcat(msg.Buf, str_end_of_line);
		osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);					// Write data on queue (In will print on StartUART_Task task)
	}
  /* USER CODE END Start_Blue_LED_Blink */
}

/* USER CODE BEGIN Header_Start_Show_Resources */
/**
* @brief Function implementing the Show_Resources thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Show_Resources */
void Start_Show_Resources(void *argument)
{
  /* USER CODE BEGIN Start_Show_Resources */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(5000);												// Every 5 second task management will print data

	  char str_end_of_line[3] = {'\r','\n'};
	  char str_sig = '-';
	  char buff[10] = {0};

	  QUEUE_t msg;												// Make a queue
	  memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'
	  strcat(msg.Buf, ">>>>> Free heap memory: ");				// Add string to another (Total heap)

	  freemem = xPortGetFreeHeapSize();							// Function return how many free memory.
	  itoa(freemem, buff, 10);
	  strcat(msg.Buf, buff);
	  strcat(msg.Buf, str_end_of_line);

	  // add a hat
	  strcat(msg.Buf, "| TASK NAME           | STATUS |   PRIOR	|  STACK  |    NUM  |\n\r\0");

	  vTaskList(str_management_memory_str);						// Fill in str_management_memory_str array management task information

	  // Finding the  end of string
	  uint16_t buffer_size = 0;
	  while(msg.Buf[buffer_size] != '\0')
	  {
	  	buffer_size ++;
	  }

	  // Add str_management_memory_str to queue string
	  int i = 0;
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }

	  // add a hat
	  char str_line[] = {"-----------------------\n\r"};
	  char str_head_2[] = {"| TASK NAME           | ABS TIME |              TASK TIME% |\n\r"};
	  strcat(msg.Buf, str_line);
	  strcat(msg.Buf, str_head_2);

	  memset(str_management_memory_str, 0, sizeof(str_management_memory_str));	// Clean buffer

	  vTaskGetRunTimeStats(str_management_memory_str);							// Function return how much time all functions running.

	  buffer_size = buffer_size + i + (sizeof(str_line)-1) + (sizeof(str_head_2)-1);           // НЕ ВИВОДИТЬ СТРОКУ !!!!!!!!!!!!!!!!!! <<<<<<<<<<<<<<<<<<<
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }
	  strcat(msg.Buf, "#########################################\n\r");

	  osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);					// Write data on queue (In will print on StartUART_Task task)
  }
  /* USER CODE END Start_Show_Resources */
}

/* USER CODE BEGIN Header_Start_UART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_Task */
void Start_UART_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_Task */
  /* Infinite loop */
  QUEUE_t msg;
  for(;;)
  {
	// osMessageQueueGet waiting data on a queue (If data are in queue so print it)
	osMessageQueueGet(UARTQueueHandle, &msg, 0, osWaitForever);			// Write for data on queue
	// Counting how many characters will be transmitted
	uint16_t buffer_size = 0;
	while(msg.Buf[buffer_size] != '\0')
	{
		buffer_size ++;
	}
	// Transmit over virtual comport
	CDC_Transmit_FS(msg.Buf, buffer_size);						// Transmit data over virtual comport
    osDelay(1);
  }
  /* USER CODE END Start_UART_Task */
}

/* USER CODE BEGIN Header_Start_bme280 */
/**
* @brief Function implementing the bme280 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_bme280 */
void Start_bme280(void *argument)
{
  /* USER CODE BEGIN Start_bme280 */
  /* Infinite loop */

	QUEUE_t msg;												// Make a queue
	//memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'

	uint16_t STATUS=0;
	uint16_t addres_device = 0x76;  		 	// BME280
	uint16_t id_addr = 0xD0;
	uint8_t id = 96;							// in hex form
	uint8_t buff=0;        						// Return 0x96 -> Dec 60

	// For debug
	STATUS = HAL_I2C_Mem_Read(&hi2c3, addres_device<<1, id_addr, 1, &buff, 1, 1000);
	if((buff == id) && (STATUS == 0))
	{
		// BME280 founded
	}
	else
	{
		// Error !!! BME280 didn't found
	}


	// Init BME280
	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);

	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	dev.delay_ms(40);

  for(;;)
  {
	  osDelay(1000);

	  char str_t_h_and_p[50] = {0};
	  char str_thp_buffer[12] = {0};

	  memset(msg.Buf, 0, sizeof(msg.Buf));								// Fill in buff '\0'
	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);		// Get data from sensor

	  if(rslt == BME280_OK)
	  {
	  		// Save data variables
	  		float BME280_temperature = comp_data.temperature;
	  		float BME280_humidity = comp_data.humidity;
	  		float BME280_preasure = comp_data.pressure;

	  		// Write T, H and P in str_t_h_and_p buffer
	  		// Write TEMPERATURE
	  		strcat(str_t_h_and_p, "BEE280: \n\r");
	  		strcat(str_t_h_and_p, "T: ");
	  		sprintf(str_thp_buffer, "%f", BME280_temperature);
	  		strcat(str_t_h_and_p, str_thp_buffer);
	  		strcat(str_t_h_and_p, " C\n\r");

	  		// Write HUMIDYTY
	  		memset(str_thp_buffer, 0, sizeof(str_thp_buffer));
	  		strcat(str_t_h_and_p, "H: ");
	  		sprintf(str_thp_buffer, "%f", BME280_humidity);
	  		strcat(str_t_h_and_p, str_thp_buffer);
	  		strcat(str_t_h_and_p, " C\n\r");

	  		// Write PRERASURE
	  		memset(str_thp_buffer, 0, sizeof(str_thp_buffer));
	  		strcat(str_t_h_and_p, "P: ");
	  		sprintf(str_thp_buffer, "%f", BME280_preasure);
	  		strcat(str_t_h_and_p, str_thp_buffer);
	  		strcat(str_t_h_and_p, " mm\n\r\0");

	  		strcat(msg.Buf, str_t_h_and_p);										//	Write main buffer with data in queue

	  		osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);			// Write data on queue (In will print on StartUART_Task task)

	  }
	  else
	  {
		  strcat(str_t_h_and_p, "ERROR!!! BME280 didn't found\n\r");
		  osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);			// Write data on queue (In will print on StartUART_Task task)
	  }

  }
  /* USER CODE END Start_bme280 */
}

/* USER CODE BEGIN Header_Start_AM2302 */
/**
* @brief Function implementing the AM2302 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AM2302 */
void Start_AM2302(void *argument)
{
  /* USER CODE BEGIN Start_AM2302 */
  /* Infinite loop */
	QUEUE_t msg;												// Make a queue
	memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'

	GPIOC->MODER |= GPIO_MODER_MODER1_0;            // Output mode GPIOC0
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_1;             // Push-pull mode
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1;     // Speed
	GPIOC->ODR ^= 0x02; 							// set GPIOC pin 1 on high
	osDelay(2000);									// First init must be 2 seconds delay

  for(;;)
  {
	  osDelay(3000);			// Measure every 3 seconds
	  //----------------------------------------------------------------------------------------
	  /*
	   * Function make us delay
	   */
//	  __STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//	  {
//	  	uint32_t test_micros = SystemCoreClock;
//	  	micros *= (SystemCoreClock / 100000) /84;
//	  	while (micros--);
//	  }

	  	//  function must use less than one time per 2-3 seconds.
	  /* Init work with sensor:
	   *
	   * From microcontroller
	   * 						            From sensor
	   * 	   Low 10 msec	      High 39 us|	80us Pull down     80us Pull up 	Start Receive data from sensor
	   * ____                     _______	|					 __________________
	   * 	   \	   	           /	   \|					/				   \
	   * 	   	\_________________/			\__________________/		 			\_______
	   *
	   * Receive '0' Bit
	   *     Low 50 us    High 26 - 28 us
	   * __                ___________
	   * 	 \			    /			\
	   * 	  \____________/			 \_
	   *
	   * Receive '1' Bit
	   *     Low 50 us             High 70 us
	   * __                ________________________
	   * 	 \			    /				         \
	   * 	  \____________/			              \_
	   */

	  	bool get_data_status = false;
	  	int j = 0;   							// Counter bytes
	  	int i = 0;								// Counter bits
	  	uint8_t data[4] = {0};					// Buffer for incoming data from sensor
	  	float temper, hum;						// Buffer variables

	  	// Init GPIO like output
	  	GPIOC->MODER |= GPIO_MODER_MODER1_0;            // Output mode GPIOC0
	  	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_1;             // Push-pull mode
	  	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1;     // Speed

	  	GPIOC->ODR &= ~0x02;		// Low level
	  	osDelay(18);
	  	GPIOC->ODR ^= 0x02;			// High level
	  	delay_us(40);

	  	// Make input pin C1
	  	GPIOC->MODER &= ~0x04;  	// Set Pin C1 Input   (MODER GPIOC_1 Must be 00)
	  	GPIOC->PUPDR &= ~0x04;		// Set Pin C1 Pull up

	  	if(GPIOC->IDR & GPIO_IDR_ID1)		// Sensor must pull down
	  	{
	  		get_data_status = false; 					// Error. Sensor not response
	  	}
	  	else
	  	{
	  		get_data_status = true;
	  	}

	  	delay_us(80);

	  	if(!(GPIOC->IDR & GPIO_IDR_ID1))  	// Sensor must pull up
	  	{
	  		get_data_status = false; 					// Error. Sensor not response
	  	}
	  	else
	  	{
	  		get_data_status = true;
	  	}
	  	delay_us(80);

	  	if(get_data_status == true)
	  	{
	  		for(j = 0; j <5; j++)							// Reading 5 bytes
	  		{
	  			data[4-j] = 0;
	  			for(i = 0; i < 8; i++)						// Reading 8 bits
	  			{
	  				while(!(GPIOC->IDR & GPIO_IDR_ID1));	// While signal is "0"
	  				delay_us(30);
	  				if(GPIOC->IDR & GPIO_IDR_ID1)			// If signal is high when wrute "1" in buffer (data[])
	  				{
	  					data[4-j] |= (1 << (7 - i));        // Shift received bite
	  				}
	  				while(GPIOC->IDR & GPIO_IDR_ID1);		// Wait end of "1" signal
	  			}
	  			get_data_status = true;										// Data was been written okay
	  		}

	  		temper = (float)((*(uint16_t*)(data+1)) & 0x3FFF) /10;
	  		if((*(uint16_t*)(data+1)) & 0x8000) temper  *= -1.0;
	  		hum = (float)(*(int16_t*)(data+3)) / 10;

	  		// Write data in queue
	  		char str_t_and_h[50] = {0};
	  		char str_t_and_h_buffer[12] = {0};

	  		memset(msg.Buf, 0, sizeof(msg.Buf));								// Fill in buff '\0'

	  		// Write T and  H P in str_t_h buffer
	  		// Write TEMPERATURE
	  		strcat(str_t_and_h, "AM2302: \n\r");
	  		strcat(str_t_and_h, "T: ");
	  		sprintf(str_t_and_h_buffer, "%f", temper);
	  		strcat(str_t_and_h, str_t_and_h_buffer);
	  		strcat(str_t_and_h, " C\n\r");

	  		// Write HUMIDYTY
	  		memset(str_t_and_h_buffer, 0, sizeof(str_t_and_h_buffer));

	  		strcat(str_t_and_h, "H: ");
	  		sprintf(str_t_and_h_buffer, "%f", hum);
	  		strcat(str_t_and_h, str_t_and_h_buffer);
	  		strcat(str_t_and_h, " C\n\r\0");

	  		strcat(msg.Buf, str_t_and_h);										//	Write main buffer with data in queue

	  		osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);			// Write data on queue (In will print on StartUART_Task task)
	  	}
	  	else
	  	{

	  		//i2c_device.AM2302_ready_status = false;
	  	}

  }



  /* USER CODE END Start_AM2302 */
}

/* USER CODE BEGIN Header_Start_SD_CARD */
/**
* @brief Function implementing the SD_CARD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SD_CARD */
void Start_SD_CARD(void *argument)
{
  /* USER CODE BEGIN Start_SD_CARD */
  /* Infinite loop */

	Mount_SD("/");

	Create_File("test_data_1.txt");
	Update_File("test_data_1.txt","\n\rStart recording\r\n");	// Add data to the end of file

	// Create folders
	Create_Dir("test_folder_1");
	Create_Dir("test_folder_2");
	Create_Dir("test_folder_3");

	Unmount_SD("/");

	static int i = 0;											// Test data for write

  for(;;)
  {
	  // Log data ewery one second
	  osDelay(1000);
	  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);			// LED ON

	  Mount_SD("/");

	  char data[10] = {0};
	  sprintf(data, "%d\n", i);
	  Update_File("test_data_1.txt", data);						// Add data to the end of file
	  i++;

	  Unmount_SD("/");

	  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);		// LED OFF



  }
  /* USER CODE END Start_SD_CARD */
}

/* USER CODE BEGIN Header_Start_LCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD */
void Start_LCD(void *argument)
{
  /* USER CODE BEGIN Start_LCD */
  /* Infinite loop */
	TFT9341_ini(240, 320);
	TFT9341_FillScreen(TFT9341_BLUE);
	uint16_t i,j;

  for(;;)
  {
	  osDelay(2000);
	  speed_test();
	  TFT9341_FillScreen(TFT9341_BLACK);






//	  osDelay(500);
//	  TFT9341_FillScreen(TFT9341_BLACK);


	//lcd_test_print();
	//HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END Start_LCD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	// Handler for generate us dalay 			( FOR AM2302 )
	if(htim->Instance == TIM10) 				//check if the interrupt comes from TIM10
	{
		if(tim_val > 0)
		{
			tim_val = tim_val - 1;
		}
		else									// For avoid overflow variable
		{
			tim_val = 0;
		}
	}

	// Handler for SD
	if(htim->Instance == TIM1) 					//check if the interrupt comes from TIM1 (Blink LED)
	{
		//HAL_GPIO_TogglePin(GPIOD, LD4_Pin);		// Green LED
	}

	// Handler for count how many time works any tasks
	if(htim->Instance == TIM3)
	{
		ulHighFreqebcyTimerTicks++;					// Update time tasks counter
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

	if (htim->Instance == TIM14)		// For SD works (use in fatfs_sd.c file)
	{
		if(Timer1 > 0)
		    Timer1--;

		  if(Timer2 > 0)
		    Timer2--;
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
