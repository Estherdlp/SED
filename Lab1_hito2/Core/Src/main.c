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

int aux = 0;			//Variables auxiliares ejecucion programa
int alarm = 0;
int options = 99;
int pulsadorSAVE = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void demora_100ms(uint32_t ciclosON, uint32_t ciclosOFF){	//Funcion retardo
	if (aux == 1){									//Ciclo LED encendido
		GPIOA -> BSRR = (1<<5);
		for (uint32_t i=0; i<ciclosON; i++){
			if (alarm == 1){
				break;
			}
		}
		aux = 0;
	}
	else {											//Ciclo LED apagado
		GPIOA -> BSRR = (1<<5) << 16;
		for (uint32_t i=0; i<ciclosOFF; i++){
			if (alarm == 1){
				break;
			}
		}
		aux = 1;
	}
}

void config_GPIO(){					//Configuracion puertos micro
	__HAL_RCC_GPIOA_CLK_ENABLE();		//Habilitacion reloj puerto A

	GPIOA -> OSPEEDR &= ~(1 <<(5*2 +1));//Velocidad de micro lenta (84 MHz)
	GPIOA -> OSPEEDR &= ~(1 << (5*2));

	GPIOA -> MODER &= ~(1 << (4*2 +1));	//Configuracion GPIOA-4 como entrada digital
	GPIOA -> MODER &= ~(1 << (4*2));	//SAVE

	GPIOA -> MODER &= ~(1 << (1*2 +1));	//Configuracion GPIOA-1 como entrada digital
	GPIOA -> MODER &= ~(1 << (1*2));	//D1

	GPIOA -> MODER &= ~(1 << (0*2 +1));	//Configuracion GPIOA-0 como entrada digital
	GPIOA -> MODER &= ~(1 << (0*2));	//D0

	GPIOA -> MODER &= ~(1 << (5*2 +1));	//Configuracion GPIOA-5 como salida digital
	GPIOA -> MODER |= (1 << (5*2));		//Salida LED placa

	GPIOA -> OTYPER &= ~(1 << 5);		//Configuracion salida push pull

	GPIOA -> PUPDR &= ~(1 << (5*2+1));	//Configuracion salida flotante
	GPIOA -> PUPDR &= ~(1 << (5*2));
}

void GPIO_InitExti(){					//Configuracion puerto interrupcion
	SYSCFG->EXTICR[1] &= ~(0x0000000F);	//[1] pines 7 a 4, 0000 corresponde a GPIOA
	EXTI->IMR |= (uint16_t) (0x1 << 4);	//Habilitacion linea de EXTI
	EXTI->RTSR &= (uint16_t) (0 << 4);	//Deshabilitacion flanco de subida
	EXTI->FTSR |= (uint16_t) (1 << 4);	//Habilitacion flanco de bajada
}

void InitNVIC(){						//Configuracion interrupciones
	NVIC_SetPriority (EXTI4_IRQn, 0); 	//Prioridad de la interrupcion
	NVIC_EnableIRQ (EXTI4_IRQn); 		//Habilitacion interrupcion
	EXTI->PR |= (1 << 4); 				//Limpiar interrupcion al inicio
}

void EXTI4_IRQHandler(){				//Metodo interrupcion al pulsar PA4 (save)
	extern int alarm;					//Invocar variables externas al ISR
	extern int pulsadorSAVE;
	if (EXTI -> PR & (1<<4)){			//Comprobacion puerto que ha producido interrupcion
		for (uint32_t j=0; j<38500; j++)//Retardo 5ms para estabilizacion de la entrada
		pulsadorSAVE = !(GPIOA -> IDR & GPIO_PIN_4);//Lectura valor pulsador SAVE
		if (pulsadorSAVE == 1){
		alarm = 1;						//Activacion alarma
		}
	}
	EXTI->PR |= (1 << 4);				//Limpiar interrupcion
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
  //HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  config_GPIO();		//Configuracion puertos
  GPIO_InitExti();		//Configuracion puertos interrupciones
  InitNVIC();			//Configuracion interrupciones
  GPIOA -> BSRR = (1<<5);//Apagar LED al inicio del programa

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		if (alarm == 1){							//Lectura botones de seleccion D1D0 al pulsar save
			aux = 1;
			if (!(GPIOA -> IDR & GPIO_PIN_1)){
				if (!(GPIOA -> IDR & GPIO_PIN_0)){	//D1D0: 0b00
					options = 1;
				}
				else{								//D1D0: 0b01
					options = 2;
				}
			}
			else{									//D1D0: 0b10
				if (!(GPIOA -> IDR & GPIO_PIN_0)){
					options = 3;
				}
				else{								//D1D0: 0b11
					options = 4;
				}
			}
			alarm = 0;								//Desactivar alarma
		}

		switch (options){
		case 1:
			demora_100ms(4000000,4000000);	//0b00, 1 Hz, DC 50%
			break;
		case 2:
			demora_100ms(2000000,6000000);	//0b01, 1 Hz, DC 25%
			break;
		case 3:
			demora_100ms(2000000,2000000);	//0b10, 2 Hz, DC 50%
			break;
		case 4:
			demora_100ms(3000000,1000000);	//0b11, 2 Hz, DC 75%
			break;
		default:
			demora_100ms(4000000,4000000);	//Default: 1 Hz, DC 50%
			break;
		}

		/* USER CODE END 3 */
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
