/*
 * NVIC.C
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <NVIC.h>

void ConfigNVIC(){							//Configuracion monitorizacion interrupciones
	NVIC_SetPriority (EXTI0_IRQn, 1); 		//Prioridad de la interrupcion PB0 (Pulso MORSE)
	NVIC_EnableIRQ (EXTI0_IRQn); 			//Desenmascarar interrupcion PB0 (Pulso MORSE)
	EXTI -> PR |= (1 << 0);					//Limpiar interrupcion al inicio

	NVIC -> ISER[1] |= (1 << 18);			//Habilitamos TIM5 IRQn (Distinguir - de . en MORSE)
	NVIC -> ISER[0] |= (1 << 30); 			//Habilitar interrupcion TIM4 IRQn (ancho PWM)
	NVIC -> ISER[0] |= (1 << 29);			//Habilitamos TIM3 IRQn (Espacios en MORSE)
}
