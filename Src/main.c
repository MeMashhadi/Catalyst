/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define HVMax (45000)
#define HVMin (0)

#define dHV (0.08)
#define HVOffset (0.0)

#define CrtMax (40.0)
#define CrtMin (0.0)

#define dCrt (100)
#define CrtOffset (0.0)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int X_counter = 0;
int step;  // stepper motor step
double Theta = 0, MaxTheta = 90;
double dTh = 0.01;

int HVvalue;
double CrtValue;

uint16_t ADC_buffer[2] = {0, 0};
char UartData[1000];
char StrCommand[1000];
char CommandStack[1000];
char StrBuffer[200];
struct Command_struct {int command_num;char Command_str[100]; int value1;int value2;int value3;};
enum direction {forward, backward};

#define size_x  600 // mm
#define step_size 0.183

int StepperPosition = 0;
const int maxstep = 6560; // max half step means 2*size_x/step_size
int delay_micro = 1000; 

char str4Send[100];
struct Command_struct CurrentCommand;
struct Command_struct MyCommand;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
int tolower(unsigned char);
void One_Step(int);
struct Command_struct str2command(char* chars);
void GoToHome(void);
void StartScan(void);
void ResetCommand(void);
double stod(char *,int);
int stoi(char *,int);
void itos(int ii, char* st);
void AddStr(char StrHost[], char NewStr[]);
void SendTime(void);
void SendDetectorValue(void);
void SendHV(void);
void SendCrt(void);
int HV2DAC(int);
int Crt2DAC(double);
void DriveStepper(int StepperPosition);
signed int StrSize(char str[]);
void SendInt(int i);
void SendDouble(double d);
void SendStr(char NewStr[]);
void AddStr(char StrHost[], char NewStr[]);
bool Cut2Enter(char StrHost[], char Command[]);
char* stristr( const char* str1, const char* str2);
void StartScan(void);
void doCommand(struct Command_struct MyCommand);
void ReceiveCommand(void);
void half_Step(enum direction dir);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	 HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	 HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	 
   step = 0;
	 for (int i=0; i<2; i++)
	 {
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		 HAL_Delay(1000);
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		 HAL_Delay(1000);
	 }
	 
	int i = 0;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_UART_Receive_DMA(&huart3,(uint8_t *)UartData,1000);
	
  while (1)
  {
		
		ReceiveCommand();
		doCommand(CurrentCommand);
	
		SendStr(CurrentCommand.Command_str);
		i++;
		
		if (i < 400)
		{
	    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
			One_Step(0);
		}
		else if(i > 800)
		{			
			i = 0;
		}
		else
		{
	    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			One_Step(1);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM8)
	{
	  	char str[100];
			X_counter = __HAL_TIM_GetCounter(&htim1);
		  HAL_TIM_Base_Stop(&htim1);
		
			sprintf(str ,"Th = %0.3f ; StepperPos = %i ; Pulse = %i \r\n", Theta, StepperPosition, X_counter);
			half_Step(forward);
   		__HAL_TIM_SetCounter(&htim1,0);
		  HAL_TIM_Base_Start(&htim1);
      
		  SendStr(str);
		  Theta += dTh;
	}
}

void DriveStepper(int StepperPosition)
{
	switch (StepperPosition)
	{
		case 0:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
		  break;
		case 1:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);			
		  break;
		case 2:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
		  break;
		case 3:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
		  break;
		case 4:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
		  break;
		case 5:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		  break;
		case 6:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
	 	  break;
		case 7:
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
 	 	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		  break;
	}
}

signed int StrSize(char str[])
{
	int size,S2,i;
	bool FindEnd;
	
	size = 0;
	S2 = 1000;
	FindEnd = false;
	
	for (i=0; i<S2; i++)
	{
		if ((str[i] == 0) & !FindEnd)
		{
			size = i;
			FindEnd = true;
		}
	}
	return size;
}

void SendInt(int i)
{
	char buffer[20];
	sprintf(buffer, "%d\r\n", i);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer,StrSize(buffer),50);
}

void SendDouble(double d)
{
	char buffer[20];
	sprintf(buffer, "%0.5f\r\n", d);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer,StrSize(buffer),50);
}

void SendStr(char NewStr[])
{
	HAL_UART_Transmit(&huart3,(uint8_t *) NewStr,StrSize(NewStr),50);
}

void AddStr(char StrHost[], char NewStr[])
{
	int i, j , k;
	
	i = StrSize(StrHost);
	j = StrSize(NewStr);
	for (k = 0; k < j; k++)
	{
		if (NewStr[k] > 0)
			StrHost[i+k] = NewStr[k];
		else
			break;
	}
	StrHost[i+j] = 0;
}

bool Cut2Enter(char StrHost[], char Command[])
{
	int i, j, k;
	bool CommandRec;
	
	CommandRec = false;
	i = StrSize(StrHost);
	
	for (k = 0; k < i; k++)
	{		
		if ((StrHost[k] > 0) && (!CommandRec))
		{
				Command[k] = StrHost[k];
				if(StrHost[k] == 10 | StrHost[k] == 13)
				{
					CommandRec = true;
				}
				Command[k+1] = 0;
		}
		else
			break;
	}
	
	if (CommandRec)
	{		
		for (j = k; j < i; j++)
		{
			StrHost[j-k] = StrHost[j];
		}
		StrHost[i-k] = 0;
		
		
		if (j < 2)
			CommandRec = false;
	}
	
	return CommandRec;
}

char* stristr( const char* str1, const char* str2 )
{
    const char* p1 = str1 ;
    const char* p2 = str2 ;
    const char* r = *p2 == 0 ? str1 : 0 ;

    while( *p1 != 0 && *p2 != 0 )
    {
        if( tolower((unsigned char)*p1) == tolower( (unsigned char)*p2 ) )
        {
            if( r == 0 )
            {
                r = p1 ;
            }
            p2++ ;
        }
        else
        {
            p2 = str2 ;
            if( r != 0 )
            {
                p1 = r + 1 ;
            }

            if( tolower( (unsigned char)*p1 ) == tolower( (unsigned char)*p2 ) )
            {
                r = p1 ;
                p2++ ;
            }
            else
            {
                r = 0 ;
            }
        }
        p1++ ;
    }

    return *p2 == 0 ? (char*)r : 0 ;
} 

void StartScan()
{
	GoToHome();
	Theta = 0;
	X_counter = 0;
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim8);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,htim8.Init.Period/2);
  __HAL_TIM_SetCounter(&htim1,0);
					
	while(Theta <= MaxTheta)
	{}
	HAL_TIM_Base_Stop(&htim8);
	HAL_TIM_Base_Stop(&htim1);
}

void doCommand(struct Command_struct MyCommand)
{
	int i;
	if (MyCommand.command_num != 0)
	{

		if (MyCommand.command_num == 1)
			GoToHome();
		else
		{
			if (MyCommand.command_num == 2)
			{
				StartScan();	
				for (i = 0;i<1000; i++)
				{
					CommandStack[i] = 0;
				}
			}
		}
	}
	ResetCommand();
}

void ReceiveCommand(void)
{
	int i;
	char myCommand[100];

	if (StrSize(UartData) > 0)
	{		
		HAL_Delay(10);		
		HAL_UART_DMAStop(&huart3);
  	AddStr(CommandStack, UartData);
		for (i=0; i<1000;i++)
			UartData[i] = 0;
		HAL_UART_Receive_DMA(&huart3,(uint8_t *)UartData,1000);
	}	
	
	if (StrSize(CommandStack) > 0){
		if (Cut2Enter(CommandStack,myCommand))
		{				
  	  CurrentCommand = str2command(myCommand);
		}		
	}
}

struct Command_struct str2command(char* chars)
{
	struct Command_struct NewCommand;
	char *pch;
	char str[30];
	
	NewCommand.command_num = 0;
	
	pch = stristr(chars,"GTH");
	if (pch!=NULL)
	{
		sprintf(NewCommand.Command_str,"GTH");	
		NewCommand.command_num = 1;
	}	
		
	pch = stristr(chars,"CON?");	
	if (pch!=NULL)
	{
		SendStr("Connected\r\n");
	}	
	
	pch = stristr(chars,"SetT");
	if (pch!=NULL)
	{	
		int ti = stoi(pch,4);
		if (ti > 400)
			delay_micro = ti;
		sprintf(str ,"T = %i \r\n",delay_micro);
		SendStr(str);	
	}
	
	pch = stristr(chars,"SHV");
	if (pch!=NULL)
	{	
		HVvalue = stoi(pch,3);
		if (HVvalue > HVMax)
		{
			sprintf(StrBuffer,"Can not Set HV to %d V . Maximum HV is %d V \r\n", HVvalue, HVMax);
			SendStr(StrBuffer);
			HVvalue = HVMax;
	  }
		if (HVvalue < HVMin )
		{
			sprintf(StrBuffer,"Can not Set HV to %d V. Minimum HV is %d V \r\n", HVvalue, HVMin);
  		SendStr(StrBuffer);
			HVvalue = HVMin;			
		}
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, HV2DAC(HVvalue));
    SendInt(HV2DAC(HVvalue));
		SendHV();
	}
	
	
	pch = stristr(chars,"SCrt");
	if (pch!=NULL)
	{	
		CrtValue = stod(pch,4);
		if (CrtValue > CrtMax )
		{
			sprintf(StrBuffer,"Can not Set Current to %0.5f mA. Maximum Current is %0.5f mA \r\n", CrtValue, CrtMax);
			SendStr(StrBuffer);
			CrtValue = CrtMax;
		}
		if (CrtValue < CrtMin )
		{	
			sprintf(StrBuffer,"Can not Set Current to %0.5f mA. Minimum Current is %0.5f mA\r\n", CrtValue, CrtMin);
			SendStr(StrBuffer);
			CrtValue = CrtMin;
	  }
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, Crt2DAC(CrtValue));
    SendInt(Crt2DAC(CrtValue));
		SendCrt();
	}
				
	
	pch = stristr(chars,"Ti?");
	if (pch!=NULL)
	{	
		SendTime();	
	}
	
	pch = stristr(chars,"HV?");
	if (pch!=NULL)
	{	
		SendHV();	
	}

	pch = stristr(chars,"Crt?");
	if (pch!=NULL)
	{	
		SendCrt();	
	}
		
	pch = stristr(chars,"Sc");
	if (pch!=NULL)
	{	
		sprintf(NewCommand.Command_str,"Sc");	
		NewCommand.command_num = 2;
	}	
	return NewCommand;
}


double stod(char *str,int start)
{
  int i,j;
	double k;
	bool IsPointRead;
	double DecimalCoeff = 1;
	
	IsPointRead = false;
  i = strlen(str);
  k=0;
	
	for(j=start;j<i;j++){
		if (!IsPointRead)
		{
			if ((str[j] >= 48) && (str[j] <= 57))
			{
				k = k * 10;
				k = k + (str[j] - 48);
			}
			else if(str[j] == 46)
			{
			  IsPointRead = true;
			}
			else
			{
				break;
			}		
		}
		else
		{
			DecimalCoeff = DecimalCoeff * 0.1;
		  if ((str[j] >= 48) && (str[j] <= 57))
			{				
				k = k + (str[j] - 48)*DecimalCoeff;
			}
			else
			{
				break;
			}		
		}
	}
	return k;
}



int stoi(char *str,int start)
{
  int i = strlen(str) ,k=0;
	
	for(int j=start;j<i;j++){
		if ((str[j] >= 48) && (str[j] <= 57))
		{
			k = k * 10;
			k = k + (str[j] - 48);
		}
		else
		{
			break;
		}
		
	}
	return k;
}

void itos(int i,char* st)
{
	int new_i;
	
	int k;
	new_i = i;

	for(int j=6; j>=0; j--){
		k = floor(new_i/pow(10,j));
		
		st[6-j] = k+48;
		new_i = new_i - k * pow(10,j);		
	}
	
	st[7] = 0;	
}

void One_Step(int dir)
{	
	//if ((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)  == GPIO_PIN_RESET) && (dir == 0)) // check the Limit sensor released. if Limit sensor checked dont move forward
	if ((dir == 0)) // check the Limit sensor released. if Limit sensor checked dont move forward
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_Delay(1);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	
	//if ((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET)  && (dir == 1)) // check the Home sensor released. if Home sensor checked dont move backward
	if (dir == 1) // check the Home sensor released. if Home sensor checked dont move backward
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	}	
}

void half_Step(enum direction dir)
{	
	if (dir == backward) 
		StepperPosition--;
	else
	  StepperPosition++;
	
	StepperPosition = (StepperPosition + 8) % 8;
	
	DriveStepper(StepperPosition);
}

void SendTime()
{
	char str[30];
	sprintf(str ,"T = %i \r\n",delay_micro);
	SendStr(str);
}

void SendHV(void)
{
	char str[30];
	sprintf(str ,"HV = %i \r\n",HVvalue);
	SendStr(str);
}
void SendCrt(void)
{
	char str[30];
	sprintf(str ,"Current = %0.5f \r\n",CrtValue);
	SendStr(str);
}

void GoToHome()
{
	uint16_t num = 0;
	
	while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) & (num < 10000)) // sensor is released
	{
		half_Step(backward);  // return to Home
		HAL_Delay(2);	
		num++;
	}
	
	if (num >= 10000)
	{	
		SendStr("Error in Goto Home.");
	}	

	
	num = 0;	
	while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) & (num < 5000))
	{
		half_Step(forward);  // return to Home
		HAL_Delay(6);	
		num++;
	}
	
	if (num >= 5000)
	{	
		SendStr("Error in Goto Home.");
		HAL_Delay(200);
	}
	
	Theta = 0;
	SendStr("In Home\r\n");
}

void ResetCommand()
{
	CurrentCommand.command_num = 0;
	sprintf(CurrentCommand.Command_str,"");
	CurrentCommand.value1 = -1;
	CurrentCommand.value2 = -1;
	CurrentCommand.value3 = -1;	
}


int HV2DAC(int HVvalue)
{
	int DACvalue;
	DACvalue = (HVvalue*dHV - HVOffset);
	if (DACvalue > 4095)
		DACvalue = 4095;
	return (DACvalue);
}

int Crt2DAC(double CrtValue)
{
	int DACvalue;
	DACvalue = (CrtValue*dCrt - CrtOffset);
	if (DACvalue > 4095)
		DACvalue = 4095;
	return (DACvalue);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
