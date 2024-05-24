/*
 * EXTI.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <EXTI.h>

void ConfigEXTI(){
	SYSCFG -> EXTICR[0] |= ((1 << 0) | (1 << 5));//[0] pines 0 a 3, GPIOB-0 y GPIOC-1

	EXTI -> IMR |= (1 << 0);				//Desenmascarar EXTI PB0 (marcha/paro bomba)
	EXTI -> RTSR &= ~(1 << 0);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 0);				//Habilitacion interrupcion flanco de bajada

	EXTI -> IMR |= (1 << 1);				//Desenmascarar EXTI PC1 (selector manual/automatico)
	EXTI -> RTSR &= ~(1 << 1);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 1);				//Habilitacion interrupcion flanco de bajada
}

