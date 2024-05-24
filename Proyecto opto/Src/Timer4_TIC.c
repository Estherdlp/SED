/*
 * Timer4_TIC.c
 *
 *  Created on: 26 abr. 2023
 *      Author: Esther
 */

#include "main.h"
#include <Timer4_TIC.h>


void ConfigTimer4(){
	RCC -> APB1ENR |= (1 << 2);				//Habilitacion reloj para Timer 4

	TIM4 -> CR1 = 0; 						//Modo autoreload deshabilitado
	TIM4 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM4 -> SMCR = 0;						//No hay modo Master-Slave

	TIM4 -> CCMR1 = 0x1;					//Modo entrada TIC, sin preescalado y sin filtrado previo

	TIM4 -> CCER = 0xB;						//Habilitacion canal 1 (CC1E), activo por ambos flancos

	TIM4 -> PSC = 84 - 1; 					//Tu 1 us
	TIM4 -> ARR = 65000 - 1;				//Duracion de la rampa para capturar las medidas del sensor: 50 ms
	TIM4 -> CNT = 0;						//Valor inicial contador

	TIM4 -> EGR = 1;						//Actualizacion registros
	TIM4 -> DIER |= (1 << 1);				//No generar interrupcion al llegar al fin de la cuenta, generar interrupcion por CC1IE

	TIM4 -> SR = 0;

	TIM4 -> CR1 |= (1 << 0);				//Arrancar timer echo sensor
}
