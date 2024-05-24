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
	GPIOA -> MODER |= (10 << (5*2));		//Configuracion funcion alternativa TIM2_CH1
	GPIOA -> AFR[0] |= (0001 << 5*4);		//Funcion alternativa 1 para PA5 (TIM2_CH1)

	GPIOA -> MODER |= (01 << (1*2));		//Configuracion GPIOA-5 como salida digital (salida LED placa)
	GPIOA -> OTYPER &= ~(1 << 1);			//Configuracion salida push pull (salida LED placa)

	//GPIOA -> MODER |= (10 << (1*2));		//Configuracion funcion alternativa TIM2_CH2
	//GPIOA -> AFR[0] |= (0001 << 1*4);		//Funcion alternativa 1 para PA1 (TIM2_CH2)
}

void InitNVIC(){
	NVIC->ISER[0] |= (1<<28); 				//Habilitamos TIM2 IRQn
}

void ConfigTimer2(){						//Parametros temporizador 2
	RCC -> APB1ENR |= (1 << 0);				//Habilitacion reloj para Timer 2

	TIM2 -> CR1 |= (1 << 7); 				//Modo autoreload habilitado
	TIM2 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM2 -> SMCR = 0;						//No hay modo Master-Slave
	TIM2 -> SR = 0;

	TIM2 -> CCER |= (1 << 0);				//Habilitacion canal 1 (CC1E)

	TIM2 -> CCMR1 = 0b01101000;				//Modo TOC, preload enable, PWM con primer semiciclo a 1, clear enable siempre a 0

	TIM2 -> PSC = 8400 - 1; 				//Base de tiempos 0.1 ms: (0.0001s*frecuencia reloj micro) - 1
	TIM2 -> ARR = 10000 - 1;				//Ancho del pulso: T/BT -> 1 seg
	TIM2 -> CCR1 = 5000 - 1;				//0.5s
	TIM2 -> CNT = 0;						//Valor inicial contador

	TIM2 -> EGR = 1;						//Actualizacion registros
	TIM2 -> DIER = 1;						//Generar interrupcion al llegar al fin de la cuenta
	TIM2 -> CR1 |= (1 << 0);				//Arrancar timer
}

void TIM2_IRQHandler(){
	if ((TIM2->SR & 0x01) == 0x01) {		//Si el timer ha llegado a fin de cuenta
		GPIOA -> ODR ^= (1 << 1); 			//Toggle led verde
		TIM2 -> SR &= ~(1 << 0); 			//Limpiar flag de fin de cuenta
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
  //HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  GPIO_CONF();
  InitNVIC();
  ConfigTimer2();
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
