/*
 * GPIO_InitExti.h
 *
 *  Created on: 5 dic. 2022
 *      Author: Esther
 */

//Configuracion EXTI boton usuario por flanco de subida

#ifndef INC_GPIO_INITEXTI_H_
#define INC_GPIO_INITEXTI_H_

void GPIO_InitExti(){						//Configuracion interrupciones via GPIO
	SYSCFG -> EXTICR[3] |= (1 << 5);		//[3] pines 12 a 15, GPIOC
	EXTI -> IMR |= (1 << 13);				//Desenmascarar EXTI pulsador boton usuario
	EXTI -> RTSR &= ~(1 << 13);				//Deshabilitacion interrupcion flanco de subida
	EXTI -> FTSR |= (1 << 13);				//Habilitacion interrupcion flanco de bajada
}

#endif /* INC_GPIO_INITEXTI_H_ */
