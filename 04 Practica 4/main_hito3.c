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
#include "stdio.h"							//Librerias para imprimir por pantalla
#include "string.h"

int valor_ADC = 0;
int pulsador = 0;
int finConversion = 0;
float tension = 0.0;
char msg_res[100];
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void GPIO_CONF(void){
	//CONFIGURACION RELOJES GPIO
	RCC -> AHB1ENR |= (1 << 2); 			//Habilitacion reloj puerto C
	RCC -> AHB1ENR |= (1 << 0);				//Habilitacion reloj puerto A

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOC -> MODER &= ~(11 << (13*2));		//Configuracion GPIOC-13 como entrada digital (boton usuario)
	GPIOA -> MODER |= (11 << 0*2);			//Configuracion PA0 como analogico
}

void GPIO_InitExti(){						//Configuracion interrupciones via GPIO
	SYSCFG -> EXTICR[3] |= (1 << 5);		//[3] pines 12 a 15, GPIOC
	EXTI -> IMR |= (1 << 13);				//Desenmascarar EXTI pulsador boton usuario
	EXTI -> RTSR &= ~(1 << 13);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 13);				//Habilitacion interrupcion flanco de bajada
}

void InitNVIC(){							//Configuracion monitorizacion interrupciones
	EXTI -> PR |= (1 << 13); 				//Limpiar interrupcion boton usuario al inicio
	NVIC_SetPriority (EXTI15_10_IRQn, 0); 	//Prioridad de la interrupcion boton usuario
	NVIC_EnableIRQ (EXTI15_10_IRQn); 		//Desenmascarar interrupcion boton usuario

	NVIC->ISER[0] |= (1 << 29); 			//Habilitamos TIM3 IRQn (iniciar conversion)
	NVIC->ISER[0] |= (1 << 18); 			//Habilitamos ADC1 IRQn (fin conversion)
}

void ConfigADC(){							//Parametros ADC
	RCC -> APB2ENR |= (1 << 8);				//Habilitacion reloj ADC1

	ADC -> CCR |= (1 << 16);				//Frecuencia < 36 MHz

	ADC1 -> CR2 &= ~(1 << 0); 				//Apagar ADC para realizar la configuracion
	ADC1 -> CR2 &= ~(1 << 1); 				//Bit CONT = 0 para conversion simple
	ADC1 -> CR2 |= (1 << 10); 				//Flag end of conversion despues de cada conversion
	ADC1 -> CR2 &= ~(1 << 11); 				//Big align a 0 para alinear los bits a la derecha
	ADC1 -> CR2 |= (1 << 28);				//External trigger enable = 1
	ADC1 -> CR2 |= (1 << 27);				//TIM3_TRGO event

	ADC1 -> CR1 |= (1 << 5); 				//Habilitada interrupcion fin conversion, resolucion de 12 bits, un solo canal, deshabilitacion de interrupcion por desbordamiento

	ADC1 -> SQR1 = 0;						//Secuencia de un elemento
	ADC1 -> SQR3 &= ~(11111 < 0);			//Canal PA0

	ADC1 -> SMPR2 &= ~(111 << 3);			//Muestrear en 3 ciclos

	ADC1 -> CR2 |= (1 << 0);				//Encender ADC una vez configurado
}

void ConfigTimer3(){						//Parametros temporizador 4 (activar conversion cada 0.5s)
	RCC -> APB1ENR |= (1 << 1);				//Habilitacion reloj para Timer 4

	TIM3 -> CR1 = (1 << 7); 				//Modo autoreload habilitado
	TIM3 -> CR2 = 0x20;			 			//Compare OC1REF trigger output
	TIM3 -> SMCR = 0;						//No hay modo Master-Slave

	TIM3 -> CCMR1 = 0x0000;					//Modo BT

	TIM3 -> CCER = 0;						//Modo BT

	TIM3 -> PSC = 8400 - 1; 				//Tu 0.1 ms
	TIM3 -> ARR = 10000 - 1;				//Tiempo para realizar una conversion: 1s
	TIM3 -> CNT = 0;						//Valor inicial contador

	TIM3 -> EGR = 1;						//Actualizacion registros
	TIM3 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM3 -> SR = 0;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void EXTI15_10_IRQHandler() {				//Interrupcion boton usuario
	extern int pulsador;
	if (EXTI -> PR & (1 << 13)) { 			//Comprobar si ha saltado la EXTI por PC13
		if (pulsador == 0){
			TIM3 -> CR1 |= (1 << 0);		//Arrancar timer 2 (realizar conversion)
			pulsador = 1;
		}
		else {
			TIM3 -> CR1 &= ~(1 << 0);		//Parar timer 2 (no realizar mas conversiones)
			pulsador = 0;
		}
	}
	EXTI -> PR |= (1 << 13); 				//Limpio el flag para la siguiente vez
}

void TIM3_IRQHandler(){						//Interrupcion timer 4 (realizar conversion)
	if ((TIM3 -> SR & 0x0001) != 0) {		//Comprobar si se ha producido interrupcion por fin de cuenta
		TIM3 -> SR = 0;						//Limpiar flag de fin de cuenta
	}
}

void ADC_IRQHandler(){						//Interrupcion ADC
	extern int finConversion;
	if ((ADC1 -> SR & (1 << 1)) != 0) {		//Comprobar si se ha producido fin conversion
		 valor_ADC = (ADC1 -> DR);			//Volcar en una variable de mas de 12 bits la conversion
		 finConversion = 1;					//Variable auxiliar para imprimir resultado
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

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
  GPIO_InitExti();
  InitNVIC();
  ConfigADC();
  ConfigTimer3();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (pulsador == 1 && finConversion == 1){
		  tension = valor_ADC * 3.3 / 4095;					//Calculo del valor de la tension
		  sprintf(msg_res, "\r\nValor de tension medido: %f", tension);
		  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		  finConversion = 0;								//Variable auxiliar imprimir resultado
	  }
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
