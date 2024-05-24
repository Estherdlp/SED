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
int pulsador = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void GPIO_CONF(void){
	//CONFIGURACION RELOJES
	__HAL_RCC_GPIOA_CLK_ENABLE(); 			//Habilitacion reloj puerto A
	__HAL_RCC_GPIOC_CLK_ENABLE(); 			//Habilitacion reloj puerto C

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOC->MODER &= ~(11 << (13*2)); 	 	//Configuracion GPIOC-13 como entrada digital (pulsador usuario)
	GPIOA->MODER |= (01 << (5*2)); 			//Configuracion GPIOA-5 como salida digital (LED micro)
	GPIOA->OTYPER &= ~(1 << (5));			//Configuracion salida led micro push pull
	GPIOA -> OSPEEDR &= ~(11 << (5*2));		//Velocidad de puertos lenta (400KHz)
}

void GPIO_InitExti(){						//Configuracion interrupcion
	SYSCFG->EXTICR[3] |= (1 << 5);			//[3] pines 12 a 15, GPIOC
	EXTI->IMR |= (uint16_t) (0x1 << 13);	//Habilitacion linea de EXTI pulsador
	EXTI->RTSR &= (uint16_t) (0 << 13);		//Deshabilitacion flanco de subida
	EXTI->FTSR |= (uint16_t) (1 << 13);		//Habilitacion flanco de bajada
}

void InitNVIC(){							//Configuracion monitor interrupciones
	EXTI->PR |= (1 << 13); 					//Limpiar interrupcion al inicio
	NVIC_SetPriority (EXTI15_10_IRQn, 0); 	//Prioridad de la interrupcion
	NVIC_EnableIRQ (EXTI15_10_IRQn); 		//Habilitacion interrupcion
	NVIC->ISER[0] |= (1<<30); 				//Habilitamos TIM4 IRQn
}

void ConfigTimer4(){						//Parametros temporizador 4
	RCC ->APB1ENR |= (1<<2);				//Reloj para Timer 4
	TIM4->CR1 = 0; 							//Modo up. ARPE=0
	TIM4->CR2 = 0;			 				//No hay modo Master-Slave
	TIM4->SMCR = 0;							//Siempre a cero
	TIM4->DIER = 1;							//Generar interrupcion al llegar al fin de la cuenta
	TIM4->CCMR1 = 0x0000;					//Modo salida contador: 0000 TOC
	TIM4->CCER = 0x0000;					//Siempre 0 en TOC
	TIM4->PSC = 32000-1; 					//Calculo de base de tiempos
	TIM4->ARR = 500-1;						//Valor maximo de cuenta
	TIM4->CNT = 0;							//Valor inicial contador
	TIM4->EGR = 1;							//Actualizacion registros
	TIM4->CR1 |= 0x01;						//Arrancar timer
}

void TIM4_IRQHandler(){
	if ((TIM4->SR & 0x01) == 0x01) {		//Es un evento de update
		GPIOA -> ODR ^=(1<<5); 				//Toggle PA5
		TIM4 -> SR &= ~(1 << 0); 			//Limpiar flag de fin de cuenta
	}
}

void EXTI15_10_IRQHandler() {
	extern int pulsador;
	if (EXTI->PR & (1<<13)) { 				//Si ha ocurrido un flanco de bajada en PC13
		EXTI->PR |= (1 << 13); 				//Limpio el flag de EXTI0 para la siguiente vez
		if (pulsador == 0){
			pulsador = 1;
			TIM4->ARR = 249;				//Modificacion limite cuenta
			TIM4->CNT = 0;					//Actualizacion contador a 0 para evitar bug
		}
		else{
			pulsador = 0;
			TIM4->ARR = 499;				//Modificacion limite cuenta
			TIM4->CNT = 0;					//Actualizacion contador a 0 para evitar bug
		}
	}
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  GPIO_CONF();
  GPIO_InitExti();
  InitNVIC();
  ConfigTimer4();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  }
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
