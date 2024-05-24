/*
 * ConfigTimer3.h
 *
 *  Created on: 5 dic. 2022
 *      Author: Esther
 */

//Timer base de tiempos modo maestro para disparo de ADC

#ifndef INC_CONFIGTIMER3_H_
#define INC_CONFIGTIMER3_H_

void ConfigTimer3(){						//Parametros temporizador 3 (activar conversion cada 0.25s)
	RCC -> APB1ENR |= (1 << 1);				//Habilitacion reloj para Timer 3

	TIM3 -> CR1 = (1 << 7); 				//Modo autoreload habilitado
	TIM3 -> CR2 = 0x20;			 			//Compare OC1REF trigger output
	TIM3 -> SMCR = 0;						//No hay modo Master-Slave

	TIM3 -> CCMR1 = 0x0000;					//Modo BT

	TIM3 -> CCER = 0;						//Modo BT

	TIM3 -> PSC = 8400 - 1; 				//Tu 0.1 ms
	TIM3 -> ARR = 2500 - 1;					//Tiempo para realizar una conversion: 0.25s
	TIM3 -> CNT = 0;						//Valor inicial contador

	TIM3 -> EGR = 1;						//Actualizacion registros
	TIM3 -> DIER |= (1 << 0);				//Generar interrupcion al llegar al fin de la cuenta

	TIM3 -> SR = 0;
}

#endif /* INC_CONFIGTIMER3_H_ */
