/*
 * ADC.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <ADC.h>

void ConfigADC(){							//Parametros ADC
	RCC -> APB2ENR |= (1 << 8);				//Habilitacion reloj ADC1

	ADC1 -> CR2 &= ~(1 << 0); 				//Apagar ADC para realizar la configuracion

	ADC -> CCR |= (1 << 16);				//PCLK/4 = 5.25 MHz
	ADC1 -> CR2 &= ~(1 << 1); 				//Bit CONT = 0 para conversion simple
	ADC1 -> CR2 |= (1 << 10); 				//Flag end of conversion despues de cada conversion
	ADC1 -> CR2 &= ~(1<<11); 				//Big align a 0 para alinear los bits a la derecha
	ADC1 -> CR2 |= (1 << 28);				//External trigger enable = 1
	ADC1 -> CR2 |= (1 << 27);				//TIM3_TRGO event

	ADC1 -> CR1 |= (1 << 5 | (1 << 8)); 	//Habilitada interrupcion fin conversion, resolucion de 12 bits, modo scan, deshabilitacion de interrupcion por desbordamiento

	ADC1 -> SQR1 |= (0b001 << 20);			//Secuencia de dos elementos
	ADC1 -> SQR3 |= (0x4 << 5);				//Canales PA0 (potenciometro) y PA4 (ldr)

	ADC1 -> SMPR2 &= ~(111 << 3);			//Muestrear en 3 ciclos

	ADC1 -> CR2 |= (1 << 0);				//Encender ADC una vez configurado
}
