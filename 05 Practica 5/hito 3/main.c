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
#include "math.h"							//Libreria para hacer el log
//Librerias para imprimir por pantalla
#include "stdio.h"
#include "string.h"

#include "GPIO_CONF.h"						//Configuracion relojes
#include "InitNVIC.h"						//Configuracion NVIC
#include "ConfigTimer2.h"					//Configuracion timer PWM
#include "ConfigTimer3.h"					//Configuracion timer disparo ADC
#include "ConfigADC.h"						//Configuracion disparo ADC

//Variables auxiliares
int menu = 0;								//Navegacion menu
int indice = 0;								//Indice mediciones ADC
int finConversion = 0;						//Fin conversiones ADC
int valor_ADC[4] = {0, 0, 0, 0};			//Guardar mediciones ADC
float media = 0;							//Media ADC para modo promedio
char datoUART [1] = "A";					//Dato recibido UART
char msg_res[200];							//Cadena emitido UART

//Variables modelo Steinhart-Hart para obtencion temperatura con NTC
float V = 0;								//Tension entrada ADC
float R_NTC = 0.0;							//Valor resistencia NTC
float logR = 0.0;							//log para calculo
float Tsh = 0.0;							//Temperatura sin disipacion
float temperatura = 0.0;					//Temperatura con disipacion
const int R = 10000;						//Resistencia del divisor
const int Vcc = 3.3;						//Tension de alimentacion NTC
const int K = 2.5e-3;						//Factor de disipacion en mW/C
	//Constantes modelo Steinhart-Hart para obtencion temperatura con NTC
const float A = 1.11492089e-3;
const float B = 2.372075385e-4;
const float C = 6.954079529e-8;



ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2){
	extern char datoUART[1];
	extern int menu;

	if (huart2 -> Instance == USART2) {		//Confirmar si la interrupcion salta por dato en usart2
		menu = atoi(datoUART);				//Guardar valor introducido en variable menu como entero
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2){
	__NOP();
}

void TIM3_IRQHandler(){						//Interrupcion timer 3 (realizar conversion)
	if ((TIM3 -> SR & 0x0001) != 0) {		//Comprobar si se ha producido interrupcion por fin de cuenta
		TIM3 -> SR = 0;						//Limpiar flag de fin de cuenta
	}
}

void ADC_IRQHandler(){						//Interrupcion ADC
	extern int finConversion;
	extern int indice;

	if ((ADC1 -> SR & (1 << 1)) != 0) {		//Comprobar si se ha producido fin conversion
		 valor_ADC[indice] = (ADC1 -> DR);	//Volcar en una variable de mas de 12 bits la conversion
		 indice++;
		 if (indice == 4){
			 finConversion = 1;				//Variable auxiliar para imprimir resultado
			 indice = 0;
		 }
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_CONF();
  InitNVIC();
  ConfigTimer2();
  ConfigTimer3();
  ConfigADC();

  /* USER CODE END 2 */


  sprintf(msg_res, "\r\nBienvenido al hito 3");
  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */
	  HAL_UART_Receive_IT(&huart2, datoUART, 1);
	  sprintf(msg_res, "\r\nPulse 1 para PWM y 2 para temperatura\r\n");		//Mensaje seleccion de modo
	  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

	  menu = -1;

	  while (menu < 0){								//Lectura opcion a utilizar
		  HAL_UART_Receive_IT(&huart2, datoUART, 1);
	  }

	  if (menu == 1){								//Modo PWM
		  while (menu != -1){
			  sprintf(msg_res, "\r\nEstas en el modo 1. Escoge un modo de PWM:\r\n   Pulse 1 para DC 75%\r\n   Pulse 2 para DC 25%\r\n   Pulse 0 para volver\r\n");
			  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

			  menu = -1;


			  while (menu < 0){						//Lectura submodo PWM
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }

			  if (menu == 0){						//Retroceder a home
				  menu = -1;

			  }

			  if (menu == 1){						//Submodo PWM 75%
				  sprintf(msg_res, "\r\nEstas en el modo PWM 75%\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

				  TIM2 -> CCER |= (1 << 4);			//Habilitacion canal 2 (CC2E)
				  TIM2 -> CCR2 = 5000 - 1;			//DC 75%
				  TIM2 -> CNT = 0;					//Resetear valor contador
				  TIM2 -> CR1 |= (1 << 0);			//Arrancar timer 2 (PWM)
			  }

			  if (menu == 2){						//Submodo PWM 25%
				  sprintf(msg_res, "\r\nEstas en el modo PWM 25%\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

				  TIM2 -> CCER |= (1 << 4);			//Habilitacion canal 2 (CC2E)
				  TIM2 -> CCR2 = 15000 - 1;			//DC 25%
				  TIM2 -> CNT = 0;					//Resetear valor contador
				  TIM2 -> CR1 |= (1 << 0);			//Arrancar timer 2 (PWM)
			  }

			  while (menu > 0){						//Retroceder a modo 1
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }
			  TIM2 -> CR1 &= ~(1 << 0); 			//Apagar timer 2 (PWM)
			  TIM2 -> CCER &= ~(1 << 4);			//Deshabilitacion canal 2 (CC2E)
		  }
	  }

	  if (menu == 2){								//Modo medir temperatura ADC
		  while (menu != -1){
			  sprintf(msg_res, "\r\nEstas en el modo 2. Escoge un modo de temperatura:\r\n   Pulse 1 para medicion simple\r\n   Pulse 2 promedio\r\n   Pulse 0 para volver\r\n");
			  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

			  menu = -1;

			  while (menu < 0){						//Lectura submodo medir temperatura ADC
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }

			  if (menu == 0){						//Retroceder a home
				  menu = -1;
			  }

			  if (menu == 1){						//Submodo medicion simple
				  sprintf(msg_res, "\r\nEstas en el modo medicion simple\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

				  TIM3 -> CR1 |= (1 << 0);			//Arrancar timer 3 (realizar conversion)

				  while (finConversion != 1){
					  __NOP();
				  }

				  finConversion = 0;

				  //Calculo de la temperatura en funcion de la tension medida en el divisor con la NTC
				  V = (float)(valor_ADC[3]) / 4095.0 * Vcc;
				  R_NTC = (R * V) / (Vcc - V);
				  logR = log(R_NTC);

				  Tsh = 1.0/(A + B * logR + C * logR * logR * logR);	//Temperatura sin tener en cuenta el autocalentamiento
				  temperatura = Tsh - V*V/(K + R_NTC);					//Temperatura teniendo en cuenta el autocalentamiento


				  sprintf(msg_res, "\r\nLa temperatura obtenida es: %f Kelvin o %f Celsius\r\n", temperatura, temperatura - 273.15);
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 50000);

				  menu = 0;
			  }

			  if (menu == 2){						//Submodo realizacion de 4 mediciones y calculo de promedio
				  sprintf(msg_res, "\r\nEstas en el modo promedio. Pulse 0 para finalizar las mediciones\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

				  TIM3 -> CR1 |= (1 << 0);			//Arrancar timer 3 (realizar conversion)

				  while (menu == 2){
					  HAL_UART_Receive_IT(&huart2, datoUART, 1);		//Leer si se quiere finalizar la medida de temperaturas

					  while (finConversion != 1){						//Esperar hasta que se han realizado 4 conversiones
						  __NOP();
					  }

					  finConversion = 0;								//Limpiar flag 4 conversiones realizadas

					  media = (valor_ADC[0] + valor_ADC[1] + valor_ADC[2] + valor_ADC[3]) / 4.0;	//Calculo media de valores obtenidos

					  //Calculo de la temperatura en funcion de la tension medida en el divisor con la NTC
					  V = media / 4095.0 * Vcc;
					  R_NTC = (R * V) / (Vcc - V);
					  logR = log(R_NTC);

					  Tsh = 1.0/(A + B * logR + C * logR * logR * logR);	//Temperatura sin tener en cuenta el autocalentamiento
					  temperatura = Tsh - V*V/(K + R_NTC);					//Temperatura teniendo en cuenta el autocalentamiento


					  sprintf(msg_res, "\r\nLa temperatura obtenida es: %f Kelvin o %f Celsius", temperatura, temperatura - 273.15);
					  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 50000);
				  }
			  }

			  while (menu > 0){						//Retroceder a modo 2
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }
			  TIM3 -> CR1 &= ~(1 << 0); 			//Apagar timer (realizar conversion)
		  }
	  }
	 /* USER CODE BEGIN 3 */
	}
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
