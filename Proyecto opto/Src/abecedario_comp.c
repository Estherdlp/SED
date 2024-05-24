/*
 * abecedario_comp.c
 *
 *  Created on: Apr 27, 2023
 *      Author: sara
 */

#include "main.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
extern char msg_res[500];

void abecedario_comp(int *valor_morse_numero){

	switch(*valor_morse_numero){
	case 2333:
		sprintf(msg_res, "\r\ne");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2233:
		sprintf(msg_res, "\r\ni");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2223:
		sprintf(msg_res, "\r\ns");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2222:
		sprintf(msg_res, "\r\nh");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2133:
		sprintf(msg_res, "\r\na");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2213:
		sprintf(msg_res, "\r\nu");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2221:
		sprintf(msg_res, "\r\nv");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2113:
		sprintf(msg_res, "\r\nw");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2111:
		sprintf(msg_res, "\r\nj");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2123:
		sprintf(msg_res, "\r\nr");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2122:
		sprintf(msg_res, "\r\nl");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2212:
		sprintf(msg_res, "\r\nf");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 2112:
		sprintf(msg_res, "\r\np");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1333:
		sprintf(msg_res, "\r\nt");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1133:
		sprintf(msg_res, "\r\nm");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1113:
		sprintf(msg_res, "\r\no");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1233:
		sprintf(msg_res, "\r\nn");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1123:
		sprintf(msg_res, "\r\ng");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1122:
		sprintf(msg_res, "\r\nz");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1121:
		sprintf(msg_res, "\r\nq");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1223:
		sprintf(msg_res, "\r\nd");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1222:
		sprintf(msg_res, "\r\nb");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1213:
		sprintf(msg_res, "\r\nk");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1212:
		sprintf(msg_res, "\r\nc");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1211:
		sprintf(msg_res, "\r\ny");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	case 1221:
		sprintf(msg_res, "\r\nx");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	default:
		sprintf(msg_res, "\r\nERROR EN LA RECEPCION");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
		break;
	}
}
