/*
 * Timer4.BT.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <Timer4_BT.h>

void ConfigTimer4(){						//Parametros temporizador 4 (tiempo conversiones modo automatico)
	RCC -> APB1ENR |= (1 << 2);				//Habilitacion reloj para Timer 4

	TIM4 -> CR1 = (1 << 7); 				//Modo autoreload habilitado
	TIM4 -> CR2 = 0;			 			//Control register 2 a 0
	TIM4 -> SMCR = 0;						//No hay modo Master-Slave

	TIM4 -> CCMR1 = 0x0000;					//Modo BT
	TIM4 -> CCER = 0;						//Modo BT

	TIM4 -> PSC = 21000 - 1; 				//Tu 1 ms
	TIM4 -> ARR = 30000 - 1;				//Tiempo para mostrar medida: 30 s

	TIM4 -> EGR = 1;						//Actualizacion registros
	TIM4 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM4 -> SR = 0;
}
