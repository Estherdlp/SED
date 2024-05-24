/*
 * NVIC.C
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <NVIC.h>

void ConfigNVIC(){							//Configuracion monitorizacion interrupciones
	NVIC_SetPriority (EXTI1_IRQn, 0); 		//Prioridad de la interrupcion PC1 (modo auto/manual)
	NVIC_EnableIRQ (EXTI1_IRQn); 			//Desenmascarar interrupcion PC1 (modo auto/manual)
	EXTI -> PR |= (1 << 1); 				//Limpiar interrupcion al inicio


	NVIC_SetPriority (EXTI0_IRQn, 1); 		//Prioridad de la interrupcion PB0 (marcha/paro bomba)
	NVIC_EnableIRQ (EXTI0_IRQn); 			//Desenmascarar interrupcion PB0 (marcha/paro bomba)
	EXTI -> PR |= (1 << 0);					//Limpiar interrupcion al inicio


	NVIC -> ISER[0] |= (1 << 30); 			//Habilitamos TIM4 IRQn (iniciar conversion modo auto)
	NVIC -> ISER[0] |= (1 << 29);			//Habilitamos TIM3 IRQn (iniciar conversion modo manual)
	NVIC -> ISER[1] |= (1 << 18);			//Habilitamos TIM5 IRQn (antirrebote)
	NVIC -> ISER[0] |= (1 << 18); 			//Habilitamos ADC1 IRQn (fin conversion)
}
