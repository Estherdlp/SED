/*
 * Setup.c
 *
 *  Created on: Dec 17, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include "stdio.h"
#include "string.h"

#include <Setup.h>
#include <LCD.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern char msg_res[500];

extern int iter;

void Setup_automatico(){
	GPIOA -> BSRR = (1 << 7);				//Enciende LED amarillo (indicador modo automatico)
	GPIOB -> BSRR = (1 << 6) << 16;			//Apaga LED azul (indicador modo manual)

	TIM4 -> CNT = 29000;					//Valor de contador alto para enviar primera medida rapido
	TIM4 -> CR1 |= (1 << 0);				//Arrancar timer 4 (tomar medidas)

	sprintf(msg_res, "\r\nModo automatico\nPara cambiar a modo manual presione el boton 1.");
	HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

	sprintf(msg_res, "\n");					//Salto de linea para evitar bugs BT
	HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
	HAL_Delay(50);
	sprintf(msg_res, "Auto\r\n");
	HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
	HAL_Delay(50);
}

void Setup_manual(){
	GPIOA -> BSRR = (1 << 7) << 16;			//Apaga LED amarillo (indicador modo automatico)
	GPIOB -> BSRR = (1 << 6);				//Enciende LED azul (indicador modo manual)
	GPIOA -> BSRR = (1 << 6) << 16;			//Apaga LED verde (indicador motor ON)
	TIM4 -> CR1 &= ~(1 << 0);				//Parar timer 4 (tomar medidas modo automatico)

	if (iter == 0){
		sprintf(msg_res, "\r\nModo manual\nPulse 0 para volver al modo automatico\r\nPulse 2 para activar la bomba");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

		lcd_clear();
		lcd_put_cur(0, 0);
		sprintf(msg_res, "Modo");
		lcd_send_string (msg_res);
		lcd_put_cur(1, 0);
		sprintf(msg_res, "Manual");
		lcd_send_string (msg_res);

		sprintf(msg_res, "Manual\r\n");
		HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
		HAL_Delay(50);

		iter = 1;
	}
}


void Setup_manual_bomba(){
	GPIOA -> BSRR = (1 << 6);				//Enciende LED verde (indicador motor ON)

	TIM2 -> CR1 |= (1 << 0);				//Arrancar timer 2 (PWM)
	TIM2 -> CCER |= (1 << 4);				//Habilitacion canal 2 (CC2E)
	GPIOB -> BSRR = (1 << 5);				//Habilitar driver motor

	TIM3 -> CR1 |= (1 << 0);				//Habilitar timer 3 (regular velocidad PWM modo manual)
}
