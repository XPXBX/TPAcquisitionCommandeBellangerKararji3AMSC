/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 1
#define CMD_BUFFER_SIZE 64
#define MAX_ARGS 9
#define ADC_BUFFER_SIZE 8
#define	NMOY 100
// LF = line feed, saut de ligne
#define ASCII_LF 0x0A
// CR = carriage return, retour chariot
#define ASCII_CR 0x0D
// DEL = delete
#define ASCII_DEL 0x7F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t prompt[]="user@Nucleo-STM32G431>>";
uint8_t started[]=
		"\r\n*-----------------------------*"
		"\r\n| Welcome on Nucleo-STM32G431 |"
		"\r\n*-----------------------------*"
		"\r\n";
uint8_t newline[]="\r\n";
uint8_t cmdNotFound[]="Command not found\r\n";
uint8_t help[];
uint8_t pinout[];
uint8_t powerOn[];
uint8_t powerOff[];
uint32_t uartRxReceived=0;
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
uint8_t ADC_buffer[ADC_BUFFER_SIZE];
uint16_t tab[NMOY];
uint8_t call = 0;
uint8_t flag=0;
uint8_t idx=0;
uint8_t statut=0;
uint8_t raw_Value;
uint8_t Average_Voltage;
uint8_t vitesse;
uint8_t courant;
int alpha;
int Alpha1;
int Alpha2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int ConversionAlpha(int vitesse);
void CCR_Alpha(int alpha);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

int ConversionAlpha(int vitesse)
{
	int ValeurAlpha = ((vitesse + 3000)/60);
	CCR_Alpha(ValeurAlpha);
	return ValeurAlpha;
}

void CCR_Alpha(int alpha)
{
	Alpha1 = (alpha*TIM1 -> ARR)/100;
	Alpha2 = TIM1 -> ARR-Alpha1;
	TIM1->CCR1 = Alpha1;
	TIM1->CCR2 = Alpha2;
	/*HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);*/
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	char	 	cmdBuffer[CMD_BUFFER_SIZE];
	int 		idx_cmd;
	int			Start;
	int 		AntiStart;
	char* 		argv[MAX_ARGS];
	int		 	argc = 0;
	char*		token;
	int 		newCmdReady = 0;
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
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	memset(argv,NULL,MAX_ARGS*sizeof(char*));
	memset(cmdBuffer,NULL,CMD_BUFFER_SIZE*sizeof(char));
	memset(uartRxBuffer,NULL,UART_RX_BUFFER_SIZE*sizeof(char));
	memset(uartTxBuffer,NULL,UART_TX_BUFFER_SIZE*sizeof(char));

	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2, started, sizeof(started), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, prompt, sizeof(prompt), HAL_MAX_DELAY);
	//demarrage des PWM attention fonction differente pour complementaire
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))
	{
		Error_Handler();
	}

	if(HAL_OK != HAL_ADC_Start_DMA(&hadc2, ADC_buffer, ADC_BUFFER_SIZE))
	{
		Error_Handler();
	}

	if(HAL_OK != HAL_TIM_Base_Start(&htim1))
	{
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{


		// uartRxReceived is set to 1 when a new character is received on uart 1
		if(uartRxReceived)
		{
			switch(uartRxBuffer[0])
			{
			// Nouvelle ligne, instruction à traiter
			case ASCII_CR: // 13 en ASCII
				HAL_UART_Transmit(&huart2, newline, sizeof(newline), HAL_MAX_DELAY);
				cmdBuffer[idx_cmd] = '\0';
				argc = 0;
				token = strtok(cmdBuffer, " ");
				while(token!=NULL)
				{
					argv[argc++] = token;
					token = strtok(NULL, " ");
				}

				idx_cmd = 0;
				newCmdReady = 1;
				break;
				// Suppression du dernier caractère
			case ASCII_DEL:   // 08 en ASCII
				cmdBuffer[idx_cmd--] = '\0';
				HAL_UART_Transmit(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
				break;
				// Nouveau caractère
			default:
				cmdBuffer[idx_cmd++] = uartRxBuffer[0];
				HAL_UART_Transmit(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);
			}
			uartRxReceived = 0;
		}


		if(newCmdReady)
		{


			if(strcmp(argv[0],"help")==0)
			{
				HAL_UART_Transmit(&huart2, "Voici la liste des commandes:\r\n", sizeof("Voici la liste des commandes:\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - help\r\n", sizeof("  - help\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Affiche la liste des commandes\r\n", sizeof("	>> Affiche la liste des commandes\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - set PA5 (0 ou 1)\r\n", sizeof("  - set PA5 (0 ou 1)\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Allume ou eteint la LED\r\n", sizeof("	>> Allume ou eteint la LED\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - pinout\r\n", sizeof("  - pinout\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Affiche la liste des PIN utilises et leurs utilisations\r\n", sizeof("	>> Affiche la liste des PIN utilises et leurs utilisations\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - start\r\n", sizeof("  - start\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Demarre la generation de PWM\r\n", sizeof("	>> Demarre la generation de PWM\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - stop\r\n", sizeof("  - stop\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Stop la generation de PWM\r\n", sizeof("	>> Stop la generation de PWM\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - alpha\r\n", sizeof("  - alpha\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Modifie la valeur du rapport cyclique Alpha entre 0 et 1\r\n", sizeof("	>> Modifie la valeur du rapport cyclique Alpha entre 0 et 1\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - reset\r\n", sizeof("  - reset\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Reinitialise le systeme\r\n", sizeof("	>> Reinitialise le systeme\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - speed = XXXX\r\n", sizeof("  - speed = XXXX\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Regle la vitesse à XXXX [-3000 a 3000] RPM\r\n", sizeof("	>> Regle la vitesse à XXXX [-3000 a 3000] RPM\r\n"), HAL_MAX_DELAY);
			}



			else if(strcmp(argv[0],"set")==0)
			{
				if(strcmp(argv[1],"PA5")==0)
				{
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, atoi(argv[2]));
					sprintf(uartTxBuffer,"Switch on/off led : %d\r\n",atoi(argv[2]));
					HAL_UART_Transmit(&huart2, uartTxBuffer, 32, HAL_MAX_DELAY);
				}
				else
				{
					HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
				}

			}


			else if(strcmp(argv[0],"pinout")==0)
			{
				HAL_UART_Transmit(&huart2, "liste des PIN utilises :\r\n", sizeof("liste des PIN utilises :\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA5  : Allumer/eteindre la LED", sizeof("	PA5  : Allumer/eteindre la LED\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA8  : PWM 1\r\n", sizeof("	PA8  : PWM 1\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA9  : PWM 2\r\n", sizeof("	PA9  : PWM 2\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA11 : PWM 1N\r\n", sizeof("	PA11 : PWM 1N\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA12 : PWM 2N\r\n", sizeof("	PA12 : PWM 2N\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PC3  : Reset\r\n", sizeof("	PC3  : Reset\r\n"), HAL_MAX_DELAY);
			}


			else if(strcmp(argv[0],"start")==0)
			{
				statut = 0;
				if (statut == 0)
				{
					Start = (50 *TIM1 -> ARR)/100;
					AntiStart = TIM1 -> ARR-Start;
					TIM1->CCR1 = Start;
					TIM1->CCR2 = AntiStart;
					HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 1);
					HAL_Delay(1);
					HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 0);
					statut = 1;
				}
			}


			else if(strcmp(argv[0],"+")==0)
			{
				TIM1->CCR1 =+2;
				TIM1->CCR2 =-2;
			}


			else if(strcmp(argv[0],"stop")==0)
			{
				statut = 1;
				if (statut == 1)
				{
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
					HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
					statut = 0;
				}

			}

			/*else if(strcmp(argv[0],"adc")==0)
			   {
				if(1==flag)
				{
					raw_Value = 0;
					for (idx=0;idx< ADC_BUFFER_SIZE;idx++)
					{
						raw_Value = raw_Value + ADC_buffer[idx];
					}
					Average_Voltage = raw_Value/ADC_BUFFER_SIZE;
					sprintf(uartTxBuffer,"Raw est lu");
					HAL_UART_Transmit(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE, HAL_MAX_DELAY);

					flag=0;

				}
			   }*/


			else if(strcmp(argv[0],"adc")==0)
			{
				printf("test adc\r\n");
				if(flag)
				{
					printf("%d\r\n", ADC_buffer);
					flag=0;
				}
			}


			else if(strcmp(argv[0],"alpha")==0)
			{
				CCR_Alpha(atoi(argv[1]));
			}


			else if(strcmp(argv[0],"speed")==0)
			{
				int speed = atoi(argv[1]);
				if (speed > 0)
				{
					if (speed > 3000)
					{
						speed = 3000;
					}
				}
				if (speed < 0)
				{
					if (speed < -3000)
					{
						speed = -3000;
					}
				}
				sprintf(uartTxBuffer,"La vitesse sera reglee sur %d RPM \r\n", speed);
				HAL_UART_Transmit(&huart2, uartTxBuffer, 64, HAL_MAX_DELAY);
				HAL_Delay(10);
				int NewAlpha = ConversionAlpha(speed);
				sprintf(uartTxBuffer,"Alpha = %d\r\n", NewAlpha);
				HAL_UART_Transmit(&huart2, uartTxBuffer, 64, HAL_MAX_DELAY);
			}


			else if (strcmp(argv[0],"reset")==0)
			{
				HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 1);
				HAL_Delay(1);
				HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 0);
			}


			else if(strcmp(argv[0],"get")==0)
			{
				HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
			}
			else
			{
				HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
			}


			HAL_UART_Transmit(&huart2, prompt, sizeof(prompt), HAL_MAX_DELAY);
			newCmdReady = 0;

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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.GainCompensation = 0;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = 5313;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1328;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.Pulse = 2656;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIMEx_EnableDeadTimePreload(&htim1);
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 210;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
	htim3.Init.Prescaler = 17000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 500;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BLUE_BUTTON_Pin */
	GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ISO_RESET_Pin */
	GPIO_InitStruct.Pin = ISO_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ISO_RESET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GREEN_LED_Pin */
	GPIO_InitStruct.Pin = GREEN_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallBack(ADC_HandleTypeDef* hadc)
{
	call++;
	float Imoy = 0;
	for (int i = 0; i < NMOY; ++i) {
		Imoy = Imoy + (float)tab[i];
	}
	Imoy = Imoy/NMOY;
	courant = ((Imoy-3100)/4096)*(12*3.3)-0.3;
}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart)
{
	uartRxReceived = 1;
	HAL_UART_Receive_IT(&huart2, uartRxBuffer, UART_RX_BUFFER_SIZE);
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
