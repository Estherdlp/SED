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
	RCC -> AHB1ENR |= (1 << 0); 			//Habilitacion reloj puerto A
	GPIOA -> OSPEEDR &= ~(11 << (5*2));		//Velocidad de puertos lenta (400KHz)

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOA -> MODER &= ~(11 << (4*2));		//Configuracion GPIOA-4 como entrada digital (Start/Stop)
	GPIOA -> MODER &= ~(11 << (1*2));		//Configuracion GPIOA-1 como entrada digital (DC 25%)
	GPIOA -> MODER &= ~(11 << (0*2));		//Configuracion GPIOA-0 como entrada digital (DC 75%)
	GPIOA -> MODER |= (01 << (7*2));		//Configuracion GPIOA-7 como salida digital
	GPIOA -> OTYPER &= ~(1 << 7);			//Configuracion salida push pull GPIOA-7

	GPIOA -> MODER |= (10 << (5*2));		//Configuracion funcion alternativa TIM2_CH1
	GPIOA -> AFR[0] |= (0001 << 5*4);		//Funcion alternativa 1 para PA5 (TIM2_CH1)
}

void GPIO_InitExti(){						//Configuracion interrupcion
	SYSCFG -> EXTICR[2] |= (0000 << 4);		//[2] pines 4 a 7, GPIOA
	EXTI -> IMR |= (uint16_t) (0x1 << 4);	//Habilitacion linea de EXTI pulsador start/stop
	EXTI -> RTSR &= (uint16_t) (0 << 4);	//Deshabilitacion flanco de subida
	EXTI -> FTSR |= (uint16_t) (1 << 4);	//Habilitacion flanco de bajada

	SYSCFG->EXTICR[1] |= (0000 << 4);		//[1] pines 0 a 3, GPIOA

	EXTI -> IMR |= (uint16_t) (0x1 << 1);	//Habilitacion linea de EXTI pulsador D1
	EXTI -> RTSR &= (uint16_t) (0 << 1);	//Deshabilitacion flanco de subida
	EXTI -> FTSR |= (uint16_t) (1 << 1);	//Habilitacion flanco de bajada

	EXTI -> IMR |= (uint16_t) (0x1 << 0);	//Habilitacion linea de EXTI pulsador D0
	EXTI -> RTSR &= (uint16_t) (0 << 0);	//Deshabilitacion flanco de subida
	EXTI -> FTSR |= (uint16_t) (1 << 0);	//Habilitacion flanco de bajada
}

void InitNVIC(){							//Configuracion monitor interrupciones
	NVIC_SetPriority (EXTI4_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_SetPriority (EXTI1_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_SetPriority (EXTI0_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_EnableIRQ (EXTI4_IRQn); 			//Habilitacion interrupcion pulsador start/stop
	NVIC_EnableIRQ (EXTI1_IRQn); 			//Habilitacion interrupcion pulsador D1
	NVIC_EnableIRQ (EXTI0_IRQn); 			//Habilitacion interrupcion pulsador D0
	EXTI -> PR |= (1 << 4); 				//Limpiar interrupcion al inicio
	EXTI -> PR |= (1 << 1); 				//Limpiar interrupcion al inicio
	EXTI -> PR |= (1 << 0); 				//Limpiar interrupcion al inicio
}

void ConfigTimer2(){						//Parametros temporizador 2
	RCC -> APB1ENR |= (1 << 0);				//Habilitacion reloj para Timer 2

	TIM2 -> CR1 |= (1 << 7); 				//Modo autoreload habilitado
	TIM2 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM2 -> SMCR = 0;						//No hay modo Master-Slave
	TIM2 -> SR = 0;

	TIM2 -> CCER |= (1 << 0);				//Habilitacion canal 1 (CC1E)

	TIM2 -> CCMR1 = 0b01101000;				//Modo TOC, preload enable, PWM con primer semiciclo a 1, clear enable siempre a 0


	TIM2 -> PSC = 840 - 1; 					//Base de tiempos 0.01 ms: (0.00001s*frecuencia reloj micro) - 1
	TIM2 -> ARR = 50000 - 1;				//Ancho del pulso: T/BT -> 0.5 seg
	TIM2 -> CCR1 = 25000 - 1;				//DC 50%
	TIM2 -> CNT = 0;						//Valor inicial contador

	TIM2 -> EGR = 1;						//Actualizacion registros
	TIM2 -> DIER = 0;						//No generar interrupcion al llegar al fin de la cuenta
}

void EXTI4_IRQHandler() {					//Interrupcion start/stop
	extern int pulsador;
	if (EXTI->PR & (1 << 4)) { 				//Comprobar flanco de bajada en pulsador start/stop
		if (pulsador == 0){
			TIM2 -> CR1 |= (1 << 0);		//Arrancar timer
			TIM2 -> CNT = 0;				//Resetear valor contador
			GPIOA -> BSRR = (1<<7);			//Encender LED rojo al pulsar start
			pulsador = 1;
		}
		else{								//Stop
			TIM2 -> CR1 &= ~(1 << 0);		//Parar timer
			TIM2 -> CNT = 0;				//Resetear valor contador
			GPIOA -> BSRR = (1<<7) << 16;	//Apagar LED rojo al pulsar stop
			pulsador = 0;
		}
	EXTI->PR |= (1 << 4); 					//Limpio el flag para la siguiente vez
	}
}

void EXTI0_IRQHandler(){					//Interrupcion D0
	if (EXTI->PR & (1 << 0)) { 				//Comprobar flanco de bajada en pulsador D0 (5 Hz)
		TIM2 -> CCR1 = 10000 - 1;			//DC 20%
		TIM2 -> CNT = 0;						//Actualizacion contador a 0 para evitar bug
	}
	EXTI->PR |= (1 << 0); 					//Limpio el flag para la siguiente vez
}

void EXTI1_IRQHandler(){					//Interrupcion D1
	if (EXTI->PR & (1 << 1)) { 				//Comprobar flanco de bajada en pulsador D1 (1 KHz)
		TIM2 -> CCR1 = 37500 - 1;			//DC 75%
		TIM2 -> CNT = 0;						//Actualizacion contador a 0 para evitar bug
	}
	EXTI->PR |= (1 << 1); 					//Limpio el flag para la siguiente vez
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
  //HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  GPIO_CONF();
  GPIO_InitExti();
  ConfigTimer2();
  InitNVIC();

  GPIOA -> BSRR = (1<<7) << 16;				//Iniciar LED rojo apagado
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
