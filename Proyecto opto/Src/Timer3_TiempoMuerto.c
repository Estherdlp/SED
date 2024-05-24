/*
 * Timer3_TiempoMuerto.c
 *
 *  Created on: 30 may. 2023
 *      Author: Esther del Pico
 */

#include "main.h"
#include <Timer3_TiempoMuerto.h>

void ConfigTimer3(){						//Parametros temporizador 5 (antirrebote)
	RCC -> APB1ENR |= (1 << 1);				//Habilitacion reloj para Timer 3

	TIM3 -> CR1 = 0; 						//Modo autoreload deshabilitado
	TIM3 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM3 -> SMCR = 0;						//No hay modo Master-Slave

	TIM3 -> CCMR1 = 0;						//Modo BT

	TIM3 -> CCER = 0;						//Modo BT

	TIM3 -> PSC = 8400 - 1; 				//Tu 0.1 ms
	TIM3 -> ARR = 30000 - 1;				//Tiempo para rellenar con vacios: 3 s
	TIM3 -> CNT = 0;						//Valor inicial contador

	TIM3 -> EGR = 1;						//Actualizacion registros
	TIM3 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM3 -> SR = 0;
}
