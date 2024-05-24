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
//Librerias necesarias
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <LCD.h>							//Pantalla LCD 16x2

//Librerias propias
#include <GPIO.h>							//Configuracion GPIO
#include <NVIC.h>							//Configuracion NVIC
#include <Timer2_PWM.h>						//Configuracion Timer 2 como PWM
#include <Timer3_TMRGO.h>					//Configuracion Timer 3 como TMRGO
#include <Timer4_BT.h>						//Configuracion Timer 4 como BT mediciones modo automatico
#include <Timer5_Antirrebote.h>				//Configuracion Timer 5 como BT antirrebote entradas
#include <ADC.h>							//Configuracion ADC
#include <EXTI.h>							//Desenmascarar EXTI modo manual/auto y marcha/paro bomba

#include <DHT20.h>							//Mediciones con sensor DHT20
#include <LDR.h>							//Mediciones con sensor LDR
#include <Imprimir.h>						//Imprimir los resultados ambos modos
#include <Setup.h>							//Inicializacion primera iteracion modos auto/manual
#include <Motor_automatico.h>				//Control PWM para modo automatico
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime = {0};				//Struct hora RTC
RTC_DateTypeDef sDate = {0};				//Struct fecha RTC

float temperatura = 0.0;					//Valor de temperatura
float humedad = 0.0;						//Valor de humedad
float lux = 0.0;							//Valor lux calculado

int menu = 0;								//Navegacion menu
int finConversion = 0;						//ADC ha realizado todas las conversiones
int indice = 0;								//Indice para contar los puertos del ADC
int pulsador = 0;							//Pulsador marcha/paro de bomba
int lecturaUART = 99;						//Lectura puertos serie para control del programa
int tiempo_mediciones = 150;				//Variable conteo impresion valores ADC en modo manual

int valor_luminosidad = 0;					//Valor raw luminosidad
int valor_potenciometro = 0;				//Valor raw potenciometro para velocidad motor modo manual
uint16_t Ancho_PWM;							//Obtencion DC modo manual

char datoUART [1] = "A";					//Dato recibido UART
char msg_res[500];							//Cadena emision UART

char tiempo[100];							//Tiempo RTC
char fecha[100];							//Fecha RTC

int arranque_automatico = 0; 				//Variable auxiliar arranque modo automatico
int arranque_manual = 0;					//Variable auxiliar arranque modo manual
int iter = 0;								//Variable auxiliar arranque modo manual
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void TIM3_IRQHandler();

void TIM4_IRQHandler();

void TIM5_IRQHandler();

void ADC_IRQHandler();

void EXTI0_IRQHandler();

void EXTI1_IRQHandler();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  ConfigGPIO();
  ConfigNVIC();
  ConfigEXTI();
  ConfigADC();
  lcd_init();
  ConfigTimer2();
  ConfigTimer3();
  ConfigTimer4();
  ConfigTimer5();

  HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);				//Fecha y hora RTC
  HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);

  HAL_UART_Receive_IT(&huart2, datoUART, 1);//Interrupcion entrada de dato desde PC
  HAL_UART_Receive_IT(&huart1, datoUART, 1);//Interrupcion entrada de dato desde BT

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  arranque_automatico = 0;
	  arranque_manual = 0;
	  iter = 0;
	  Parar_motor();

	  while (menu == 0){					//Modo automatico

		  if (arranque_automatico == 0){	//Setup primera iteracion en modo automatico
			  Setup_automatico();
			  arranque_automatico = 1;
		  }

		  while (finConversion != 1){		//Esperar hasta que se hayan cumplido los 30 segundos entre medida y medida
			  if (lecturaUART == 1){
				  lecturaUART = 99;
				  menu = 1;
			  }
			  if (menu == 1){
				  break;
			  }
		  }
		  //Obtener fecha y hora RTC
		  HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
		  sprintf(fecha,"\r\n%02d/%02d/%02d",sDate.Date,sDate.Month,sDate.Year);
		  HAL_UART_Transmit(&huart2, (char*) fecha, strlen(fecha), 100);
		  HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		  sprintf(tiempo," - %02d:%02d:%02d", sTime.Hours,sTime.Minutes,sTime.Seconds);
		  HAL_UART_Transmit(&huart2, (char*) tiempo, strlen(tiempo), 100);

		  Medir_Temperatura(&temperatura);	//Llamada funcion calculo temperatura
		  Medir_Humedad(&humedad);			//Llamada funcion calculo humedad
		  Medir_lux(&lux);					//Llamada funcion calculo lux

		  Imprimir_automatico(&temperatura, &humedad, &lux);	//Llamada funcion imprimir calculos T, HR, L

		  finConversion = 0;				//Cambiar valor variable auxiliar para esperar hasta proxima conversion

		  if ((sTime.Minutes % 2 != 0) && (temperatura > 12.0) && (humedad < 60.0) && (lux < 30000)){
			  Marcha_motor(&temperatura, &humedad, &lux);	//Llamada funcion calculo velocidad motor en modo automatico
		  }
		  else{								//Apagar motor si no se cumplen las condiciones
			  Parar_motor();
		  }
	  }

	  while (menu == 1){					//Modo manual

		  if (arranque_manual == 0){		//Inicializacion modo manual
			  Setup_manual();
			  Parar_motor();
			  arranque_manual = 1;
		  }

		  while (pulsador == 0){			//Lectura de todas las vias de control del sistema
			  if (lecturaUART == 0){		//Cambio a modo automatico desde usart
				  menu = 0;
				  lecturaUART = 99;
			  }
			  if (lecturaUART == 2){		//Marcha de la bomba desde usart
				  pulsador = 1;
				  lecturaUART = 99;
			  }
			  if (menu == 0){				//Cambio a modo automatico desde botonera
				  break;
			  }
		  }

		  while (pulsador == 1){			//Marcha de bomba activa
			  if (arranque_manual == 1){	//Inicializacion bomba activa
				  Setup_manual_bomba();
				  arranque_manual = 2;
			  }

			  while (finConversion != 1){	//Esperar hasta que se hayan cumplido los 100 ms entre medida y medida
				  if (lecturaUART == 0){	//Comprobar si se desea volver al modo automatico desde USART
					  pulsador = 0;
					  menu = 0;
					  lecturaUART = 99;
				  }
				  if (lecturaUART == 2){	//Comprobar si se desea parar la bomba desde USART
					  lecturaUART = 99;
					  pulsador = 0;
					  break;
				  }
				  if (menu == 0){			//Cambio a modo automatico desde botonera
					  break;
				  }
			  }
			  if (tiempo_mediciones == 300){		//Mostrar cada 30s en modo manual los resultados
				  Medir_Temperatura(&temperatura);	//Llamada funcion calculo temperatura
				  Medir_Humedad(&humedad);			//Llamada funcion calculo humedad
				  Medir_lux(&lux);					//Llamada funcion calculo lux

				  Imprimir_automatico(&temperatura, &humedad, &lux);//Llamada funcion imprimir calculos T, HR, L
				  tiempo_mediciones = 0;
			  }
			  Ancho_PWM = (uint16_t)(5000.0 + (3.68 * valor_potenciometro));	//Calculo de CCR para mover motor a diferentes velocidades
			  TIM2 -> CCR2 = Ancho_PWM - 1;			//DC PWM control motor
			  finConversion = 0;
		  }
		  Parar_motor();							//Deshabilitacion PWM, timer, enable driver y piloto ON si se desea parar la bomba
		  arranque_manual = 0;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x19;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	extern char datoUART[1];
	extern int lecturaUART;

	if (huart -> Instance == USART2) {		//Confirmar si la interrupcion salta por dato en usart2 (PC)
		if (strcmp(datoUART,"\r") == 0 || (strcmp(datoUART,"\n")) == 0){ //No hacer nada si se detecta por salto de linea/retorno de carro
			__NOP();
		}
		else {
			lecturaUART = atoi(datoUART);	//Guardar valor introducido en variable menu como entero
		}
		HAL_UART_Receive_IT(&huart2, datoUART, 1);//Interrupcion entrada de dato desde PC
	}

	if (huart -> Instance == USART1) {		//Confirmar si la interrupcion salta por dato en usart1 (BT)
		if (strcmp(datoUART,"\r") == 0 || (strcmp(datoUART,"\n")) == 0){ //No hacer nada si se detecta por salto de linea/retorno de carro
			__NOP();
		}
		else {
			lecturaUART = atoi(datoUART);	//Guardar valor introducido en variable menu como entero
		}
		HAL_UART_Receive_IT(&huart1, datoUART, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	__NOP();
}


void TIM3_IRQHandler(){						//Interrupcion timer 3 (tomar medidas modo manual)
	if ((TIM3 -> SR & 0x0001) != 0) {		//Comprobar si se ha producido interrupcion por fin de cuenta
		TIM3 -> SR = 0;						//Limpiar flag de fin de cuenta
	}
}

void TIM4_IRQHandler(){						//Interrupcion timer 4 (tomar medidas modo automatico)
	if ((TIM4 -> SR & 0x0001) != 0) {		//Comprobar si se ha producido interrupcion por fin de cuenta
		ADC1 -> CR2 |= (1 << 30);			//Activar ADC para tomar medidas en modo continuo
		TIM4 -> SR = 0;						//Limpiar flag de fin de cuenta
	}
}

void TIM5_IRQHandler(){						//Interrupcion timer 5 (antirrebote botonera)
	extern int pulsador;
	extern int menu;
	if ((TIM5 -> SR & 0x0001) != 0) {		//Comprobar si ha saltado la interrupcion por fin de cuenta
		TIM5 -> CR1 &= ~(1 << 0);			//Apagar timer antidebouncing

		if( ((GPIOC -> IDR) & GPIO_PIN_1) == 0){	//Comprobar si el boton para cambio manual/automatico esta pulsado
			if (menu == 0){					//Cambio a modo manual
				menu = 1;
			}
			else {							//Cambio a modo automatico
				menu = 0;
				pulsador = 0;
			}
		}

		if( ((GPIOB -> IDR) & GPIO_PIN_0) == 0){	//Comprobar si el boton para encender/apagar la bomba esta pulsado
			if (pulsador == 0){				//Arrancar bomba si esta pulsado
				pulsador = 1;
			}
			else {
				pulsador = 0;				//Descartar arranque de bomba si no esta pulsado
				arranque_manual = 0;
			}
		}
		TIM5 -> SR = 0;
	}
}

void ADC_IRQHandler(){						//Interrupcion ADC
	extern int valor_potenciometro;
	extern int valor_luminosidad;
	extern int finConversion;
	extern int tiempo_mediciones;

	if ((ADC1 -> SR & (1 << 1)) != 0) {		//Comprobar si se ha producido fin conversion
		if (indice == 0){
			valor_potenciometro = (ADC1 -> DR);	//Volcar en una variable de mas de 12 bits (valor potenciometro)
			indice = 1;
		}

		else {
			valor_luminosidad = (ADC1 -> DR);	//Volcar en una variable de mas de 12 bits (valor LDR)
			finConversion = 1;				//Variable auxiliar para imprimir resultado
			indice = 0;
			tiempo_mediciones++;
		}
	}
}

void EXTI0_IRQHandler(){					//Marcha/paro de bomba
	extern int menu;
	if (EXTI -> PR & (1 << 0)){
		if (menu == 1){						//Si se esta en el modo manual, encender bomba
			TIM5 -> CR1 |= (1 << 0);		//Arrancar timer antirrebote para comprobar si arrancar la bomba
		}
		EXTI -> PR |= (1 << 0);
	}
}

void EXTI1_IRQHandler(){					//Cambio entre modo manual y modo automatico
	if (EXTI -> PR & (1 << 1)){
		TIM5 -> CR1 |= (1 << 0);			//Arrancar timer antirrebote para comprobar si se cambia entre manual y automatico
		EXTI -> PR |= (1 << 1);
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
