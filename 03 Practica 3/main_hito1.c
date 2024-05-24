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
#include "math.h"
UART_HandleTypeDef huart2;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char msg_res[100];
int medicion = 0;
float captura1 = 0;
float captura2 = 0;
float distancia;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void GPIO_CONF(void){
	//CONFIGURACION RELOJES GPIO
	RCC -> AHB1ENR |= (1 << 0); 			//Habilitacion reloj puerto A
	RCC -> AHB1ENR |= (1 << 1); 			//Habilitacion reloj puerto B
	RCC -> AHB1ENR |= (1 << 2); 			//Habilitacion reloj puerto C

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOC -> MODER &= ~(11 << (13*2));		//Configuracion GPIOC-13 como entrada digital (boton usuario)

	GPIOB -> MODER |= (10 << (4*2));		//Configuracion funcion alternativa TIM3_CH1 (trigger sensor)
	GPIOB -> AFR[0] |= (0x2 << (4*4));		//Funcion alternativa 2 para PB4 (TIM3_CH1)

	GPIOB -> MODER |= (10 << (6*2));		//Configuracion funcion alternativa TIM4_CH1 (echo sensor)
	GPIOB -> AFR[0] |= (0x2 << (6*4));		//Funcion alternativa 2 para PB6 (TIM4_CH1)
}

void GPIO_InitExti(){						//Configuracion interrupciones via GPIO
	SYSCFG -> EXTICR[3] |= (1 << 5);		//[3] pines 12 a 15, GPIOC
	EXTI -> IMR |= (1 << 13);				//Desenmascarar EXTI pulsador boton usuario
	EXTI -> RTSR &= ~(1 << 13);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 13);				//Habilitacion interrupcion flanco de bajada
}

void InitNVIC(){							//Configuracion monitorizacion interrupciones
	EXTI->PR |= (1 << 13); 					//Limpiar interrupcion boton usuario al inicio
	NVIC_SetPriority (EXTI15_10_IRQn, 0); 	//Prioridad de la interrupcion boton usuario
	NVIC_EnableIRQ (EXTI15_10_IRQn); 		//Desenmascarar interrupcion boton usuario

	NVIC->ISER[0] |= (1 << 30); 			//Habilitar interrupcion TIM4 IRQn (echo del sensor)
	NVIC->ISER[1] |= (1 << 18);				//Habilitar interrupcion TIM5 IRQn (antirrebote)
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void ConfigTimer3(){						//Parametros temporizador 3 (generador de Trigger)
	RCC -> APB1ENR |= (1 << 1);				//Habilitacion reloj para Timer 3

	TIM3 -> CR1 |= (1 << 7); 				//Modo one pulse
	TIM3 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM3 -> SMCR = 0;						//No hay modo Master-Slave
	TIM3 -> SR = 0;

	TIM3 -> CCMR1 = 0b01101000;				//Modo TOC, preload enable, PWM con primer semiciclo a 1, clear enable siempre a 0

	TIM3 -> CCER |= (1 << 0);				//Habilitacion canal 1 (CC1E)

	TIM3 -> PSC = 84 - 1; 					//Tu 1 us
	TIM3 -> ARR = 40000 - 1;				//Ancho entre medida y medida: 40 ms
	TIM3 -> CCR1 = 20 - 1;					//Ancho de pulso para iniciar medida: 20 us
	TIM3 -> CNT = 0;						//Valor inicial contador

	TIM3 -> EGR = 1;						//Actualizacion registros
	TIM3 -> DIER = 0;						//No generar interrupcion al llegar al fin de la cuenta
}

void ConfigTimer4(){						//Parametros temporizador 4 (captura de Echo)
	RCC -> APB1ENR |= (1 << 2);				//Habilitacion reloj para Timer 4

	TIM4 -> CR1 = 0; 						//Modo autoreload deshabilitado
	TIM4 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM4 -> SMCR = 0;						//No hay modo Master-Slave

	TIM4 -> CCMR1 = 0x1;					//Modo entrada TIC, sin preescalado y sin filtrado previo

	TIM4 -> CCER = 0xB;						//Habilitacion canal 1 (CC1E), activo por ambos flancos

	TIM4 -> PSC = 84 - 1; 					//Tu 1 us
	TIM4 -> ARR = 65000 - 1;				//Duracion de la rampa para capturar las medidas del sensor: 50 ms
	TIM4 -> CNT = 0;						//Valor inicial contador

	TIM4 -> EGR = 1;						//Actualizacion registros
	TIM4 -> DIER |= (1 << 1);				//No generar interrupcion al llegar al fin de la cuenta, generar interrupcion por CC1IE

	TIM4 -> SR = 0;

	TIM4 -> CR1 |= (1 << 0);				//Arrancar timer echo sensor
}

void ConfigTimer5(){						//Parametros temporizador 5 (antirrebote)
	RCC -> APB1ENR |= (1 << 3);				//Habilitacion reloj para Timer 5

	TIM5 -> CR1 = 0; 						//Modo autoreload deshabilitado
	TIM5 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM5 -> SMCR = 0;						//No hay modo Master-Slave

	TIM5 -> CCMR1 = 0;						//Modo BT

	TIM5 -> CCER = 0;						//Modo BT

	TIM5 -> PSC = 8400 - 1; 				//Tu 0.1 ms
	TIM5 -> ARR = 1000 - 1;					//Tiempo para comprobar si la entrada es valida: 150 ms
	TIM5 -> CNT = 0;						//Valor inicial contador

	TIM5 -> EGR = 1;						//Actualizacion registros
	TIM5 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM5 -> SR = 0;
}
void EXTI15_10_IRQHandler() {				//Interrupcion boton usuario
	extern int medicion;
	if (EXTI -> PR & (1 << 13)) { 			//Comprobar si ha saltado la EXTI por PC13
		medicion = 1;
		TIM5 -> CR1 |= (1 << 0);			//Arrancar timer antidebouncing
	}
	else {
		__NOP ();							//Descartar si no se ha cumplido el tiempo de rebote
	}
	EXTI -> PR |= (1 << 13); 				//Limpio el flag para la siguiente vez
}

void TIM4_IRQHandler(){
	extern float captura1;
	extern float captura2;
	if ((TIM4 -> SR & 0x0002) != 0) {				//Comprobar si se ha producido captura por la entrada echo
		if( ((GPIOB -> IDR) & GPIO_PIN_6) != 0){	//Comprobar si la entrada esta a nivel alto (flanco de subida)
			captura1 = TIM4 -> CCR1;				//Guardar el valor de la cuenta. Se limpia flag en la lectura
		}
		if ( ((GPIOB -> IDR) & GPIO_PIN_6) == 0){	//Comprobar si la entrada esta a nivel bajo (flanco de bajada)
			captura2 = TIM4 -> CCR1;				//Guardar el valor de la cuenta. Se limpia flag en la lectura
		}
	}
}

void TIM5_IRQHandler(){
	if ((TIM5 -> SR & 0x0001) != 0) {				//Comprobar si ha saltado la interrupcion por fin de cuenta
		TIM5 -> CR1 & ~(1 << 0);					//Apagar timer antidebouncing
		if( ((GPIOC -> IDR) & GPIO_PIN_13) == 0){	//Comprobar si el boton de usuario esta pulsado
			TIM3 -> CR1 |= (1 << 0);				//Si el boton de usuario sigue pulsado, iniciar medicion
		}

		else {
			__NOP ();								//Descartar inicio de medicion si no esta pulsado
		}
		TIM5 -> SR = 0;
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
  /* USER CODE BEGIN 2 */
  GPIO_CONF();
  GPIO_InitExti();
  InitNVIC();
  ConfigTimer3();
  ConfigTimer4();
  ConfigTimer5();

  /* USER CODE END 2 */
  sprintf(msg_res, "\r\nEjercicio con sensor HC SR-04 con IC: ");
  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
  sprintf(msg_res, "\r\nPulse boton de usuario para realizar una medicion: ");
  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if (medicion == 1){
		medicion = 0;
		distancia = fabs(captura2 - captura1)/59;
		if (distancia >= 400){
			sprintf(msg_res, "\r\nError en la medicion. Por favor, presione el boton de usuario para realizar una nueva medida");
		}
		else {
			sprintf(msg_res, "\r\nLa distancia en cm es: %f",distancia);
		}
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
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
