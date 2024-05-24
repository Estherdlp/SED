/*
 * InitNVIC.h
 *
 *  Created on: 5 dic. 2022
 *      Author: Esther
 */

//Desenmascarar interrupciones por TMR3GO y por fin conversion ADC

#ifndef INC_INITNVIC_H_
#define INC_INITNVIC_H_

void InitNVIC(){							//Configuracion monitorizacion interrupciones
	NVIC->ISER[0] |= (1 << 29); 			//Habilitamos TIM3 IRQn (iniciar conversion)
	NVIC->ISER[0] |= (1 << 18); 			//Habilitamos ADC1 IRQn (fin conversion)
}

#endif /* INC_INITNVIC_H_ */
