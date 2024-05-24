/*
 * Timer2_PWM.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <Timer2_PWM.h>

void ConfigTimer2(){						//Parametros temporizador 2 (generador PWM canal 2)
	RCC -> APB1ENR |= (1 << 0);				//Habilitacion reloj para Timer 2

	TIM2 -> CR1 |= (1 << 7); 				//Modo autoreload habilitado
	TIM2 -> CR2 = 0;			 			//Control register 2 a 0
	TIM2 -> SMCR = 0;						//No hay modo Master-Slave
	TIM2 -> SR = 0;

	TIM2 -> CCMR1 = 0b0110100000000000;		//Modo TOC, preload enable, PWM con primer semiciclo a 1, clear enable siempre a 0

	TIM2 -> PSC = 21 - 1; 					//Tu 1 us
	TIM2 -> ARR = 20000 - 1;				//Frecuencia 50 Hz
	TIM2 -> CCR2 = 5000 - 1;				//Pulso inicial 25% DC
	TIM2 -> CNT = 0;						//Valor inicial contador 0

	TIM2 -> EGR = 1;						//Actualizacion registros
	TIM2 -> DIER = 0;						//No generar interrupcion al llegar al fin de la cuenta
}
