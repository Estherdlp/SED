/*
 * GPIO.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <GPIO.h>

void ConfigGPIO(){
	//CONFIGURACION RELOJES GPIO
	RCC -> AHB1ENR |= (1 << 1); 			//Habilitacion reloj puerto B

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOB -> MODER &= ~(11 << (0*2));		//Configuracion PB0 como entrada (Pulso MORSE)

	GPIOB -> MODER |= (10 << (3*2));		//Configuracion funcion alternativa TIM2_CH2 (PWM)
	GPIOB -> AFR[0] |= (0001 << 3*4);		//Funcion alternativa 1 para PB3 (TIM2_CH2)

	GPIOB -> MODER |= (10 << (6*2));		//Configuracion funcion alternativa TIM4_CH1 (ancho PWM)
	GPIOB -> AFR[0] |= (0x2 << (6*4));		//Funcion alternativa 2 para PB6 (TIM4_CH1)
}
