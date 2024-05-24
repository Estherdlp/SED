/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <EXTI.h>							//Desenmascarar EXTI modo manual/auto y marcha/paro bomba
#include <NVIC.h>							//Configuracion NVIC
#include <Timer5_Antirrebote.h>				//Configuracion Timer 5 como BT antirrebote entradas
#include <Timer2_PWM.h>						//Configuracion Timer 2 como PWM
#include <Timer4_TIC.h>						//Configuracion Timer 2 como PWM
#include <GPIO.h>							//Configuracion GPIO
#include <abecedario_comp.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float captura1 = 0;
float captura2 = 0;
int finLectura = 0;

char msg_res[200];							//Tamaño mensaje transmitido
char caracter;

int posicion = 0;
int valor_lectura = 0;
int Boton = 0;
int valor_morse_numero = 0;
int valor_morse[4] = {0,0,0,0};
int valor_comparativa_letras[4] = {0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI0_IRQHandler();
void TIM5_IRQHandler();
void TIM4_IRQHandler();
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
  ConfigGPIO();
  ConfigNVIC();
  ConfigEXTI();
  ConfigTimer2();
  ConfigTimer3();
  ConfigTimer4();
  ConfigTimer5();

  sprintf(msg_res, "\r\nBienvenido");
  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	  if (Boton == 1){
		  Boton = 0;

		  if (finLectura == 2){
			  if((valor_morse[posicion] >= 0) && (valor_morse[posicion] <= 50)){
				  valor_comparativa_letras[posicion] = 1;
			  }
			  if((valor_morse[posicion] >= 51) && (valor_morse[posicion] <= 100)){
				  valor_comparativa_letras[posicion] = 2;
			  }
			  if((valor_morse[posicion] < 0) || (valor_morse[posicion] > 100)){
				  valor_comparativa_letras[posicion] = 3;
			  }
			  finLectura = 0;
		  }

		  posicion = posicion + 1;
		  if (posicion > 3){
			  posicion = 0;
			  TIM3 -> CR1 &= ~(1 << 0);			//Apagar timer
			  valor_morse_numero = valor_comparativa_letras[0]*1000 + valor_comparativa_letras[1]*100 + valor_comparativa_letras[2]*10 + valor_comparativa_letras[3];

			  abecedario_comp(&valor_morse_numero);
		  }
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
void EXTI0_IRQHandler(){					//(Pulso MORSE)
	if (EXTI -> PR & (1 << 0)){
		TIM5 -> CR1 |= (1 << 0);			//Arrancar timer pulso MORSE
		TIM3 -> CR1 |= (1 << 0);
		EXTI -> PR |= (1 << 0);
	}
}

void TIM3_IRQHandler(){						//Interrupcion timer 5 (pulso morse)
	extern int posicion;
	extern int valor_comparativa_letras[];
	extern int Boton;

	int i = 0;

	if ((TIM3 -> SR & 0x0001) != 0) {		//Comprobar si ha saltado la interrupcion por fin de cuenta
		TIM3 -> CR1 &= ~(1 << 0);			//Apagar timer


		if (posicion < 4){
			for (i = posicion; i < 4; i++){
				valor_comparativa_letras[i] = 3;
				posicion = posicion + 1;
				Boton = 1;
			}
		}
		TIM3 -> SR = 0;						//Limpiar registro interrupcion
	}
}

void TIM4_IRQHandler(){
	extern float captura1;
	extern float captura2;
	extern int posicion;
	extern int valor_lectura;
	extern int finLectura;
	extern int Boton;

	if ((TIM4 -> SR & 0x0002) != 0) {				//Comprobar si se ha producido captura por la entrada echo
		if( ((GPIOB -> IDR) & GPIO_PIN_6) != 0){	//Comprobar si la entrada esta a nivel alto (flanco de subida)
			captura1 = TIM4 -> CCR1;				//Guardar el valor de la cuenta. Se limpia flag en la lectura
			finLectura = finLectura + 1;
		}
		if ( ((GPIOB -> IDR) & GPIO_PIN_6) == 0){	//Comprobar si la entrada esta a nivel bajo (flanco de bajada)
			captura2 = TIM4 -> CCR1;				//Guardar el valor de la cuenta. Se limpia flag en la lectura
			finLectura = finLectura + 1;
		}
	}

	if (finLectura == 2){
		valor_lectura = (captura2 - captura1);
		Boton = 1;

		if (posicion == 0){
			valor_morse[posicion] = valor_lectura;	//Volcar en array el ancho del pulso de morse
		}
		if (posicion == 1){
			valor_morse[posicion] = valor_lectura;	//Volcar en array el ancho del pulso de morse
		}
		if (posicion == 2){
			valor_morse[posicion] = valor_lectura;	//Volcar en array el ancho del pulso de morse
		}
		if (posicion == 3){
			valor_morse[posicion] = valor_lectura;	//Volcar en array el ancho del pulso de morse
		}

	}

}

void TIM5_IRQHandler(){						//Interrupcion timer 5 (pulso morse)
	if ((TIM5 -> SR & 0x0001) != 0) {		//Comprobar si ha saltado la interrupcion por fin de cuenta
		TIM5 -> CR1 &= ~(1 << 0);			//Apagar timer
		TIM3 -> CNT = 0;

		if ( ((GPIOB -> IDR) & GPIO_PIN_0) == 0){ //Comprobar si el boton sigue pulsado, indicar raya
			TIM2 -> CCR2 = 25;				//Ancho de pulso 75 us (raya)
		}
		if( ((GPIOB -> IDR) & GPIO_PIN_0) != 0) {								//Si el pulsador no esta pulsado, indica punto
			TIM2 -> CCR2 = 75;				//Ancho de pulso 25 us (punto)
		}
		TIM2 -> CR1 |= (1 << 0);
		TIM5 -> SR = 0;						//Limpiar registro interrupcion
	}
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
