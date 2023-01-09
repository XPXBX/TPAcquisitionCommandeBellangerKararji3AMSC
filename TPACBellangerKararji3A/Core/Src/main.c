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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
uint8_t raw_Value;
uint8_t Average_Voltage;
uint8_t vitesse;
float courant;
int alpha;
int Alpha1;
int Alpha2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int ConversionAlpha(int vitesse);
void CCR_Alpha(int alpha);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  This function is for making work printf
 * @retval ch
 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}


/**
 * @brief  CCR_Alpha changes the duty cycle of PWM
 * @argument alpha
 * @retval none
 */
void CCR_Alpha(int alpha)
{
	Alpha1 = (alpha*TIM1 -> ARR)/100;
	Alpha2 = TIM1 -> ARR-Alpha1;
	TIM1->CCR1 = Alpha1;
	TIM1->CCR2 = Alpha2;
}

/**
 * @brief Configuration to start the generation of PWM
 * @retval none
 */
void Start_PWM(void){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief Configuration to stop the generation of PWM
 * @retval none
 */
void Stop_PWM(void){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief  CCR_Alpha changes the duty cycle of PWM
 * @argument alpha
 * @retval none
 */
int ConversionAlpha(int vitesse)
{
	int ValeurAlpha = ((vitesse + 3000)/60);
	CCR_Alpha(ValeurAlpha);
	return ValeurAlpha;
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

				Start_PWM();
				CCR_Alpha(50);

			}

			else if(strcmp(argv[0],"+")==0)
			{
				TIM1->CCR1 =+2;
				TIM1->CCR2 =-2;
			}

			else if(strcmp(argv[0],"stop")==0)
			{

				Stop_PWM();

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


			else if(strcmp(argv[0],"ValCourant")==0)
			{
				sprintf(uartTxBuffer,"Courant = %0.4f",courant);
				HAL_UART_Transmit(&huart2, uartTxBuffer, sizeof("Courant = XXXXX\r\n"), HAL_MAX_DELAY);
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
