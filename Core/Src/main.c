/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include <string.h>
#include <stdio.h>
#include <stm32l4xx_ll_usart.h>
#include "queue.h"
#include "command.h"
#include <stdlib.h>
#include "temperature.h"
#include "battery.h"
#include "tsl237.h"
#include "flash.h"
#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMCLOCK   32000000
#define PRESCALAR  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;
queue_t qbuff;
uint8_t maxcommbuffsize = 50;
uint8_t combuff[50];
uint8_t ld3_state = 0;
typedef struct command {
  char * cmd_string;
  void (*cmd_function)(char * arg);
} command_t;
RTC_TimeTypeDef sysTime = {0};
RTC_DateTypeDef sysDate = {0};
RTC_AlarmTypeDef sAlarm = {0};
uint8_t toggleflag =0;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;
flash_status_t fs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int execute_command(uint8_t * line);
int parse_command (uint8_t *line, uint8_t **command, uint8_t **args);

void help_command(char *arguments);
void lof_command(char *arguments);
void lon_command(char *arguments);
void test_command(char *arguments);
void ds_command(char *arguments);
void ts_command(char *arguments);
void temp_command(char *arguments);
void battery_command(char *arguments);
void lightsensor_command(char *arguments);
void sample_command(char * arguments);
void ef_command(char *args);
void fetchcommand(uint8_t* combuff);
void toggleLED();
void gettime();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

command_t commands[] = {
  {"help",help_command},
  {"lof",lof_command},
  {"lon",lon_command},
  {"test",test_command},
  {"ds",ds_command},
  {"ts",ts_command},
  {"temp",temp_command},
  {"battery",battery_command},
  {"tsl237",lightsensor_command},
  {"sample",sample_command},
  {"log",log_command},
  {"data",data_command},
  {"ef",ef_command},
  {0,0}
};
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  init_queue(&qbuff);

  HAL_ADC_DeInit(&hadc1);
  HAL_TIM_Base_DeInit(&htim2);
  HAL_TIM_IC_DeInit(&htim2);
  flash_write_init(&fs);
  flash_write_init(&fs);
  write_log_data(&fs,"Note: Cold Reset");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  uint8_t status = fetchcomm(combuff);
	  if(status==1){
		  printf("\r\n");
		  if(execute_command(combuff)!=0){
			  printf("NOK\r\n");
			  gettime();
			  commandprompt(&sysTime,&sysDate);
		  }else{
//			  printf("OK\r\n");
			  gettime();
			  commandprompt(&sysTime,&sysDate);
		  }
	  }else if(status==2){
		  printf("NOK\r\n");
		  gettime();
		  commandprompt(&sysTime,&sysDate);
	  }

	  if (toggleflag  == 1){
		  toggleflag=0;
		  toggleLED();
	  }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
//  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
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
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  	sAlarm.AlarmTime.Hours = 0;
	sAlarm.AlarmTime.Minutes = 0;
	sAlarm.AlarmTime.Seconds = 30;
	sAlarm.AlarmTime.SubSeconds = 0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
								|RTC_ALARMMASK_MINUTES;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 1;
	sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  LL_USART_EnableIT_RXNE(USART2);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
	int execute_command(uint8_t * line) {
	  uint8_t *cmd;
	  uint8_t *arg;
	  command_t *p = commands;
	  int success = 0;

	  char comarr[10];
	  char argarr[20];

	  cmd = (uint8_t *)comarr;
	  arg = (uint8_t *)argarr;

	  if (!line) {
		return (-1); // Passed a bad pointer
	  }
	  if (parse_command(line,&cmd,&arg) == -1) {
		printf("Error with parse command");
		return (-1);
	  }
	  while (p->cmd_string) {
		if (!strcmp(p->cmd_string,(char *) cmd)) {
		  if (!p->cmd_function) {
			return (-1);
		  }
		  (*p->cmd_function)((char *)arg);            // Run the command with the passed arguments
		  success = 1;
		  break;
		}
		p++;
	  }
	  if (success) {
		return (0);
	  }
	  else {
		return (-1);
	  }
	}


	int parse_command (uint8_t *line, uint8_t **command, uint8_t **args){

        uint8_t i=0,ln=(uint8_t)strlen((char*)line);

        while(i<ln){
            if(line[i]==','){
                break;
            }
            i++;
        }

        char cmd[i+1];
        cmd[i]='\0';
        strncpy(cmd,(char*)line,i);
        strcpy((char*)*command,cmd);

        if(ln==i){
        	char argts[1];
        	argts[0]='\0';
        	strcpy((char*)*args,argts);
        }else{
        	char argts[ln-i];
        	argts[ln-i-1]='\0';
        	strncpy(argts,(char*)&line[i+1],ln-i);
        	strcpy((char*)*args,argts);
        }


    	return 1;

	}

	void test_command(char *arguments) {
		uint8_t i=0,pt=0,ln=(uint8_t)strlen((char*)arguments);
		char arg[50];
		while(i<ln){
			if(arguments[i]==','){
				if(i==ln-1){
					return;
				}
			 arg[pt]='\0';
			 printf("%s\r\n",arg);
			 pt=0;
			 i++;
			 continue;
		  }
		   arg[pt]=arguments[i];
		   pt++;
		   i++;
		}
	     printf("%s\r\n",arg);
	     printf("OK\r\n");
	  }

	void help_command(char* arguments) {
		  command_t* p = commands;
		  printf("Available Commands: \r\n");
	//	  HAL_UART_Transmit(&huart2, (uint8_t*)avail, strlen(avail), HAL_MAX_DELAY);
		  while (p->cmd_string) {
		    printf("%s\r\n",p->cmd_string);
		    p++;
		  }
		  printf("OK\r\n");
		}

	void lof_command(char* arguments)
	{
	  if (ld3_state == 1)
	  {
	    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	    ld3_state = 0;

	  }
	  printf("OK\r\n");
//	  HAL_UART_Transmit(&huart2, (uint8_t*)END_LINE, strlen(END_LINE), HAL_MAX_DELAY);
	}

	void lon_command(char* arguments)
	{
	  if (ld3_state == 0)
	  {
	    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	    ld3_state = 1;
	  }
	  printf("OK\r\n");
//	  HAL_UART_Transmit(&huart2, (uint8_t*)END_LINE, strlen(END_LINE), HAL_MAX_DELAY);
	}

	void gettime(){
		if(HAL_RTC_GetTime(&hrtc, &sysTime,RTC_FORMAT_BIN)==HAL_OK && HAL_RTC_GetDate(&hrtc,&sysDate,RTC_FORMAT_BIN==HAL_OK)){
			return;
		}
	}

	void ds_command(char *arguments){
		uint8_t i=0,pt=2,ln=(uint8_t)strlen((char*)arguments);
		char buff[15];
		buff[0]='0';
		buff[1]='x';
		uint8_t ctr=1, params[3]={-1,-1,-1};
		while(i<ln){
			if(arguments[i]==','){
				if(i==ln-1){
					printf("NOK\r\n");
					return;
				}
				buff[pt]='\0';
				if(ctr==1){
					params[ctr-1]=(uint8_t)strtol(buff,NULL,0);
				}else{
					params[ctr-1]=(uint8_t)atoi(buff);
				}

//				printf("%s %d\r\n",buff,params[ctr-1]);
//				buff[0]='0';
//				buff[1]='x';
				pt=0;
				i++;
				ctr++;
				continue;
			}

			buff[pt]=arguments[i];
		    pt++;
		    i++;
		}

		if(ctr<3){
			printf("NOK\r\n");
			return;
		}

		buff[pt]='\0';
		params[ctr-1]=(uint8_t)atoi(buff);
//		printf("%s %d\r\n",buff,params[ctr-1]);
		if((params[0]>0x12 || params[0]<0x1) || (params[1]>31 || params[1]<1) || (params[2]>99 || params[2]<0)){
			printf("NOK\r\n");
			return;
		}

		sysDate.Month=params[0];
		sysDate.Year=params[2];
//		sysDate.Year=0x23;
		sysDate.Date=params[1];

	   if (HAL_RTC_SetDate(&hrtc, &sysDate, RTC_FORMAT_BCD) != HAL_OK)
	   {
		   printf("NOK\r\n");
		   return;
	   }


	   printf("OK\r\n");
	}


	void ts_command(char *arguments){
		uint8_t i=0,pt=0,ln=(uint8_t)strlen((char*)arguments);
		char buff[15];
//		buff[0]='0';
//		buff[1]='x';
		uint8_t ctr=1, params[3]={-1,-1,-1};
		while(i<ln){
			if(arguments[i]==','){
				if(i==ln-1){
					printf("NOK\r\n");
					return;
				}
				buff[pt]='\0';
				params[ctr-1]=(uint8_t)atoi(buff);
//				printf("%s %d\r\n",buff,params[ctr-1]);
//				buff[0]='0';
//				buff[1]='x';
				pt=0;
				i++;
				ctr++;
				continue;
			}

			buff[pt]=arguments[i];
		    pt++;
		    i++;
		}

		if(ctr<3){
			printf("NOK\r\n");
			return;
		}

		buff[pt]='\0';

		params[ctr-1]=(uint8_t)atoi(buff);
//		printf("%s %d\r\n",buff,params[ctr-1]);
		if((params[0]>23 || params[0]<0) || (params[1]>59 || params[1]<0) || (params[2]>59 || params[2]<0)){
			printf("NOK\r\n");
			return;
		}

		sysTime.Hours=params[0];
		sysTime.Seconds=params[2];
		sysTime.Minutes=params[1];
//		sysDate.Year=0x23;


	   if (HAL_RTC_SetTime(&hrtc, &sysTime, RTC_FORMAT_BIN) != HAL_OK)
	   {
		   printf("NOK\r\n");
		   return;
	   }


	   printf("OK\r\n");
	}

	void temp_command(char *arguments) {
		if(arguments && strcmp(arguments,"")!=0){
			printf("NOK\n\r");
			return;
		}
	    HAL_ADC_Init(&hadc1);
	    while (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK);
	    printf("%d C\n\r",(int)read_temp());
	    HAL_ADC_DeInit(&hadc1);
	    printf("OK\n\r");

	}

	void battery_command(char *arguments) {
		if(arguments && strcmp(arguments,"")!=0){
			printf("NOK\n\r");
			return;
		}
	    HAL_ADC_Init(&hadc1);
	    while (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK);
	    printf("%.2f V\n\r",((float) read_vrefint())/1000);
	    HAL_ADC_DeInit(&hadc1);
	    printf("OK\n\r");

	}
	void lightsensor_command(char *arguments){
		if(arguments && strcmp(arguments,"")!=0){
			printf("NOK\n\r");
			return;
		}
		HAL_TIM_Base_Init(&htim2);
		HAL_TIM_IC_Init(&htim2);
		HAL_Delay(3);
		uint32_t raw;
		float frequency = 0,refClock = TIMCLOCK/(PRESCALAR);

		raw = tsl237_readsensor();

		HAL_TIM_Base_DeInit(&htim2);
		HAL_TIM_IC_DeInit(&htim2);

		frequency = refClock/raw;
		printf("%.2f Hz\n\r",frequency);
		printf("OK\n\r");
		return;
	}

void sample_command(char * arguments) {
  // if (arguments) {
  //   printf("NOK\n\r");
  //   return;
  // }

  HAL_ADC_Init(&hadc1);
  while (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK);
  uint16_t temp = (uint16_t)read_temp();
  uint16_t v = (uint16_t)read_vrefint();
  HAL_ADC_DeInit(&hadc1);

  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_IC_Init(&htim2);
  HAL_Delay(3);
  uint32_t raw;
  float frequency = 0,refClock = TIMCLOCK/(PRESCALAR);
  raw = tsl237_readsensor();
  HAL_TIM_Base_DeInit(&htim2);
  HAL_TIM_IC_DeInit(&htim2);
  frequency = refClock/raw;
  int freq_int = (int)frequency;
  write_sensor_data(&fs,v,temp,freq_int);
  printf("OK\n\r");
}

void ef_command(char *args) {
  if (!args) {
    printf("NOK\n\r");
    return;
  }
  else {
    if (!strcmp("all",args)) {
      flash_reset(&fs);
//      printf("Erase Here!\n\r");
    }
    else {
      printf("NOK\n\r");
      return;
    }
  }
  printf("OK\n\r");
}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			tsl237_done = 1;
		}
	}

	void toggleLED(){
		lon_command(NULL);
//		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(100);
		lof_command(NULL);

	}

	void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
	{
		toggleflag=1;

		if (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	}

/* USER CODE END 4 */

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

