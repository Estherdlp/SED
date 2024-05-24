/*
 * EXTI.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <EXTI.h>

void ConfigEXTI(){
	SYSCFG -> EXTICR[0] |= (1 << 0);		//[0] pines 0 a 3, GPIOB-0

	EXTI -> IMR |= (1 << 0);				//Desenmascarar EXTI PB0 (Pulso MORSE)
	EXTI -> RTSR &= ~(1 << 0);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 0);				//Habilitacion interrupcion flanco de bajada
}

