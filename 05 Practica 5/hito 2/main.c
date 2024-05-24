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
#include "stdio.h"							//Librerias para imprimir por pantalla
#include "string.h"
#include "GPIO_CONF.h"						//Configuracion relojes
#include "ConfigTimer2.h"					//Configuracion timer PWM

int menu = 0;								//Variables auxiliares
char datoUART [1] = "A";					//Almacenar caracteres recibidos por UART
char msg_res[200];							//Tamaño mensaje transmitido
UART_HandleTypeDef huart2;
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
		memset(datoUART, '\0', 100);		//Limpiar el valor de la variable que lee el bus
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2){
	__NOP();
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
  ConfigTimer2();
  /* USER CODE END 2 */
  sprintf(msg_res, "\r\nBienvenido al hito 2");
  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */
	  HAL_UART_Receive_IT(&huart2, datoUART, 1);
	  sprintf(msg_res, "\r\nPulse 1 para PWM y 2 para temperatura\r\n");		//Mensaje seleccion de modo
	  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);

	  menu = -1;

	  while (menu < 0){								//Lectura opcion a utilizar
		  HAL_UART_Receive_IT(&huart2, datoUART, 1);
	  }

	  if (menu == 1){								//Modo PWM
		  while (menu != -1){
			  sprintf(msg_res, "\r\nEstas en el modo 1. Escoge un modo de PWM:\r\nPulse 1 para DC 75%\r\nPulse 2 para DC 25%\r\nPulse 0 para volver\r\n");
			  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);

			  menu = -1;


			  while (menu < 0){						//Lectura submodo PWM
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }

			  if (menu == 0){						//Retroceder a home
				  menu = -1;

			  }

			  if (menu == 1){						//Submodo PWM 75%
				  sprintf(msg_res, "\r\nEstas en el modo PWM 75%\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);

				  TIM2 -> CCER |= (1 << 4);			//Habilitacion canal 2 (CC2E)
				  TIM2 -> CCR2 = 5000 - 1;			//DC 75%
				  TIM2 -> CNT = 0;					//Resetear valor contador
				  TIM2 -> CR1 |= (1 << 0);			//Arrancar timer
			  }

			  if (menu == 2){						//Submodo PWM 25%
				  sprintf(msg_res, "\r\nEstas en el modo PWM 25%\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);

				  TIM2 -> CCER |= (1 << 4);			//Habilitacion canal 2 (CC2E)
				  TIM2 -> CCR2 = 15000 - 1;			//DC 25%
				  TIM2 -> CNT = 0;					//Resetear valor contador
				  TIM2 -> CR1 |= (1 << 0);			//Arrancar timer
			  }

			  while (menu > 0){						//Retroceder a modo 1
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }
			  TIM2 -> CR1 &= ~(1 << 0); 			//Apagar timer PWM
			  TIM2 -> CCER &= ~(1 << 4);			//Deshabilitacion canal 2 (CC2E)
		  }
	  }

	  if (menu == 2){								//Modo medir temperatura ADC
		  while (menu != -1){
			  sprintf(msg_res, "\r\nEstas en el modo 2. Escoge un modo de temperatura:\r\nPulse 1 para medicion simple\r\nPulse 2 promedio\r\nPulse 0 para volver\r\n");
			  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);

			  menu = -1;

			  while (menu < 0){						//Lectura submodo medir temperatura ADC
				  HAL_UART_Receive_IT(&huart2, datoUART, 1);
			  }

			  if (menu == 0){						//Retroceder a home
				  menu = -1;
			  }

			  if (menu == 1){						//Submodo medicion simple
				  sprintf(msg_res, "\r\nEstas en el modo medicion simple\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);
				  menu = 0;
			  }

			  if (menu == 2){						//Submodo realizacion de 4 mediciones y calculo de promedio
				  sprintf(msg_res, "\r\nEstas en el modo promedio\r\nPulse 0 para volver\r\n");
				  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 200);
				  menu = 0;
			  }
		  }
	  }
	  /* USER CODE BEGIN 3 */

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
