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
	GPIOA -> OSPEEDR &= ~(11 << (5*2));		//Velocidad de puertos lenta (400KHz)

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOA -> MODER &= ~(11 << (4*2));		//Configuracion GPIOA-4 como entrada digital (Start/Stop)
	GPIOA -> MODER &= ~(11 << (1*2));		//Configuracion GPIOA-1 como entrada digital (1K Hz)
	GPIOA -> MODER &= ~(11 << (0*2));		//Configuracion GPIOA-0 como entrada digital (5 Hz)
	GPIOA -> MODER |= (01 << (5*2));		//Configuracion GPIOA-5 como salida digital (salida LED placa)
	GPIOA -> OTYPER &= ~(1 << 5);			//Configuracion salida push pull (salida LED placa)
}

void GPIO_InitExti(){						//Configuracion interrupcion
	SYSCFG->EXTICR[2] |= (0000 << 4);		//[2] pines 4 a 7, GPIOA
	EXTI->IMR |= (uint16_t) (0x1 << 4);		//Habilitacion linea de EXTI pulsador start/stop
	EXTI->RTSR &= (uint16_t) (0 << 4);		//Deshabilitacion flanco de subida
	EXTI->FTSR |= (uint16_t) (1 << 4);		//Habilitacion flanco de bajada

	SYSCFG->EXTICR[1] |= (0000 << 4);		//[1] pines 0 a 3, GPIOA

	EXTI->IMR |= (uint16_t) (0x1 << 1);		//Habilitacion linea de EXTI pulsador D1
	EXTI->RTSR &= (uint16_t) (0 << 1);		//Deshabilitacion flanco de subida
	EXTI->FTSR |= (uint16_t) (1 << 1);		//Habilitacion flanco de bajada

	EXTI->IMR |= (uint16_t) (0x1 << 0);		//Habilitacion linea de EXTI pulsador D0
	EXTI->RTSR &= (uint16_t) (0 << 0);		//Deshabilitacion flanco de subida
	EXTI->FTSR |= (uint16_t) (1 << 0);		//Habilitacion flanco de bajada
}

void InitNVIC(){							//Configuracion monitor interrupciones
	NVIC_SetPriority (EXTI4_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_SetPriority (EXTI1_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_SetPriority (EXTI0_IRQn, 0); 		//Prioridad de la interrupcion
	NVIC_EnableIRQ (EXTI4_IRQn); 			//Habilitacion interrupcion pulsador start/stop
	NVIC_EnableIRQ (EXTI1_IRQn); 			//Habilitacion interrupcion pulsador D1
	NVIC_EnableIRQ (EXTI0_IRQn); 			//Habilitacion interrupcion pulsador D0
	NVIC->ISER[0] |= (1<<30); 				//Habilitamos TIM4 IRQn
	EXTI->PR |= (1 << 4); 					//Limpiar interrupcion al inicio
	EXTI->PR |= (1 << 1); 					//Limpiar interrupcion al inicio
	EXTI->PR |= (1 << 0); 					//Limpiar interrupcion al inicio
}

void ConfigTimer4(){						//Parametros temporizador 4 -- OJO 16 bits para calculo BT!!!
	RCC ->APB1ENR |= (1<<2);				//Habilitacion reloj para Timer 4
	TIM4->CR1 = 0; 							//Modo up. ARPE=0
	TIM4->CR2 = 0;			 				//No hay modo Master-Slave
	TIM4->SMCR = 0;							//Siempre a cero
	TIM4->DIER = 1;							//Generar interrupcion al llegar al fin de la cuenta
	TIM4->CCMR1 = 0x0000;					//Modo salida contador: 0000 TOC
	TIM4->CCER = 0x0000;					//Siempre 0 en TOC

	TIM4->PSC = (84000/2) - 1; 				//Base de tiempos 0.5 ms: (1ms/ mitad frecuencia reloj micro) - 1 -> 0.001*84e6/2 - 1
	TIM4->ARR = 2000-1;						//Nº de ms que debe contar para 1s: 2000

	TIM4->CNT = 0;							//Valor inicial contador
	TIM4->EGR = 1;							//Actualizacion registros

}

void TIM4_IRQHandler(){
	if ((TIM4->SR & 0x01) == 0x01) {		//Si el timer ha llegado a fin de cuenta
		GPIOA -> ODR ^=(1<<5); 				//Toggle led micro
		TIM4 -> SR &= ~(1 << 0); 			//Limpiar flag de fin de cuenta
	}
}

void EXTI4_IRQHandler() {					//Interrupcion start/stop
	extern int pulsador;
	if (EXTI->PR & (1 << 4)) { 				//Comprobar flanco de bajada en pulsador start/stop
		if (pulsador == 0){
			TIM4->CR1 |= 0x01;				//Arrancar timer
			TIM4->CNT = 0;					//Resetear valor contador
			GPIOA -> BSRR = (1<<5);			//Encender LED micro al pulsar start
			pulsador = 1;
		}
		else{								//Stop
			TIM4->CR1 &= ~(1 << 0);			//Parar timer
			TIM4->CNT = 0;					//Resetear valor contador
			GPIOA -> BSRR = (1<<5) << 16;	//Apagar LED micro al pulsar stop
			pulsador = 0;
		}
	EXTI->PR |= (1 << 4); 					//Limpio el flag para la siguiente vez
	}
}

void EXTI0_IRQHandler(){					//Interrupcion D0
	if (EXTI->PR & (1 << 0)) { 				//Comprobar flanco de bajada en pulsador D0 (5 Hz)
		TIM4->ARR = 400-1;					//Nº de ms que debe contar para 0.2s: 0.2/0.0005 - 1 = 400 - 1
		TIM4->CNT = 0;						//Actualizacion contador a 0 para evitar bug
	}
	EXTI->PR |= (1 << 0); 					//Limpio el flag para la siguiente vez
}

void EXTI1_IRQHandler(){					//Interrupcion D1
	if (EXTI->PR & (1 << 1)) { 				//Comprobar flanco de bajada en pulsador D1 (1 KHz)
		TIM4->ARR = 2-1;					//Nº de ms que debe contar para 0.001s: 0.001/0.0005 - 1 = 2 - 1
		TIM4->CNT = 0;						//Actualizacion contador a 0 para evitar bug
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
  ConfigTimer4();
  InitNVIC();

  GPIOA -> BSRR = (1<<5) << 16;		//Iniciar LED micro apagado
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
