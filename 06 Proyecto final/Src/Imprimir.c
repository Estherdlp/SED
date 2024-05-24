/*
 * Imprimir.c
 *
 *  Created on: Dec 11, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include "stdio.h"
#include "string.h"

#include <Imprimir.h>
#include <LCD.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern char msg_res[500];


float Imprimir_automatico(float *temperatura, float *humedad, float *lux){
	sprintf(msg_res, "\r\nL: %.1f lux\r\nTemperatura: %.1f C\r\nHumedad: %.1f %%", *lux, *temperatura, *humedad);	//Imprimir valores medidos PC
	HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 1000);

	lcd_clear();												//Imprimir valores medidos LCD 16x2
	lcd_put_cur(0, 0);
	sprintf(msg_res, "L: %.1f T: %.1f", *lux, *temperatura);
	lcd_send_string (msg_res);
	lcd_put_cur(1, 0);
	sprintf(msg_res, "HR: %.1f %%", *humedad);
	lcd_send_string (msg_res);

	sprintf(msg_res, "%.1f lux\r", *lux);						//Imprimir valores medidos BT
	HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
	HAL_Delay(50);	//Delay para poder mandar por BT
	sprintf(msg_res, "%.1f C\r\n", *temperatura);
	HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
	HAL_Delay(50);	//Delay para poder mandar por BT
	sprintf(msg_res, "HR: %.1f\r\n", *humedad);
	HAL_UART_Transmit(&huart1, (char *) msg_res, strlen(msg_res), HAL_MAX_DELAY);
	HAL_Delay(50);	//Delay para poder mandar por BT
}
