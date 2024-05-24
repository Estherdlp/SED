/*
 * Timer3_TMRGO.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <Timer3_TMRGO.h>

void ConfigTimer3(){						//Parametros temporizador 3 (TMRGO modo manual)
	RCC -> APB1ENR |= (1 << 1);				//Habilitacion reloj para Timer 3

	TIM3 -> CR1 = (1 << 7); 				//Modo autoreload habilitado
	TIM3 -> CR2 = 0x20;			 			//Compare OC1REF trigger output
	TIM3 -> SMCR = 0;						//No hay modo Master-Slave

	TIM3 -> CCMR1 = 0x0000;					//Modo BT
	TIM3 -> CCER = 0;						//Modo BT

	TIM3 -> PSC = 2100 - 1; 				//Tu 0.1 ms
	TIM3 -> ARR = 1000 - 1;					//Tiempo para realizar una conversion: 100 ms
	TIM3 -> CNT = 0;						//Valor inicial contador

	TIM3 -> EGR = 1;						//Actualizacion registros
	TIM3 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM3 -> SR = 0;
}
