/*
 * ConfigTimer2.h
 *
 *  Created on: Dec 6, 2022
 *      Author: Esther
 */

#ifndef INC_CONFIGTIMER2_H_
#define INC_CONFIGTIMER2_H_

void ConfigTimer2(){						//Parametros temporizador 2 (PWM motor)
	RCC -> APB1ENR |= (1 << 0);				//Habilitacion reloj para Timer 2

	TIM2 -> CR1 |= (1 << 7); 				//Modo autoreload habilitado
	TIM2 -> CR2 = 0;			 			//Control register 2: siempre a 0
	TIM2 -> SMCR = 0;						//No hay modo Master-Slave
	TIM2 -> SR = 0;

	TIM2 -> CCMR1 = 0b0111100000000000;		//Modo TOC, preload enable, PWM con primer semiciclo a 0, clear enable siempre a 0

	TIM2 -> CCER |= (1 << 4);				//Habilitacion canal 2 (CC2E)

	TIM2 -> PSC = 84 - 1; 					//Tu 1 us
	TIM2 -> ARR = 20000 - 1;				//Ancho del pulso: T/Tu -> 0.02 seg
	TIM2 -> CCR2 = 15000 - 1;				//DC 75%
	TIM2 -> CNT = 0;						//Valor inicial contador

	TIM2 -> EGR = 1;						//Actualizacion registros
	TIM2 -> DIER = 1;						//Generar interrupcion al llegar al fin de la cuenta
}

#endif /* INC_CONFIGTIMER2_H_ */
