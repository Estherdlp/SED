/*
 * Timer5_Antirrebote.c
 *
 *  Created on: Dec 18, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <Timer5_Antirrebote.h>

void ConfigTimer5(){						//Parametros temporizador 5 (antirrebote)
	RCC -> APB1ENR |= (1 << 3);				//Habilitacion reloj para Timer 5

	TIM5 -> CR1 = 0; 						//Modo autoreload deshabilitado
	TIM5 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM5 -> SMCR = 0;						//No hay modo Master-Slave

	TIM5 -> CCMR1 = 0;						//Modo BT

	TIM5 -> CCER = 0;						//Modo BT

	TIM5 -> PSC = 2100 - 1; 				//Tu 0.1 ms
	TIM5 -> ARR = 1000 - 1;					//Tiempo para comprobar si la entrada es valida: 100 ms
	TIM5 -> CNT = 0;						//Valor inicial contador

	TIM5 -> EGR = 1;						//Actualizacion registros
	TIM5 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM5 -> SR = 0;
}
