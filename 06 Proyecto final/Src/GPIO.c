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
	RCC -> AHB1ENR |= (1 << 2); 			//Habilitacion reloj puerto C
	RCC -> AHB1ENR |= (1 << 1); 			//Habilitacion reloj puerto B
	RCC -> AHB1ENR |= (1 << 0);				//Habilitacion reloj puerto A

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOB -> MODER &= ~(11 << (0*2));		//Configuracion PB0 como entrada (marcha/paro bomba)
	GPIOC -> MODER &= ~(11 << (1*2));		//Configuracion PC1 como entrada (selector manual/automatico)

	GPIOA -> MODER |= (11 << 0*2);			//Configuracion PA0 como analogico (potenciometro velocidad motor)
	GPIOA -> MODER |= (11 << 4*2);			//Configuracion PA4 como analogico (LDR)

	GPIOB -> MODER |= (01 << (5*2));		//Configuracion GPIOB-5 como salida digital (EN giro motor)
	GPIOA -> MODER |= (01 << 7*2);			//Configuracion GPIOA-6 como salida (LED amarillo, modo automatico)
	GPIOB -> MODER |= (01 << 6*2);			//Configuracion GPIOB-6 como salida (LED azul, modo manual)
	GPIOA -> MODER |= (01 << 6*2);			//Configuracion GPIOA-7 como salida (LED verde, bomba encendida)

	GPIOB -> MODER |= (10 << (3*2));		//Configuracion funcion alternativa TIM2_CH2 (PWM)
	GPIOB -> AFR[0] |= (0001 << 3*4);		//Funcion alternativa 1 para PB3 (TIM2_CH2)
}
