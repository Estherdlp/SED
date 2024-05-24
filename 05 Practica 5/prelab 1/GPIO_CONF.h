/*
 * GPIO_CONF.h
 *
 *  Created on: 5 dic. 2022
 *      Author: Esther
 */

//Configuracion relojes y entradas/salidas

#ifndef INC_GPIO_CONF_H_
#define INC_GPIO_CONF_H_

void GPIO_CONF(){
	//CONFIGURACION RELOJES GPIO
	RCC -> AHB1ENR |= (1 << 2); 			//Habilitacion reloj puerto C
	RCC -> AHB1ENR |= (1 << 0);				//Habilitacion reloj puerto A

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOC -> MODER &= ~(11 << (13*2));		//Configuracion GPIOC-13 como entrada digital (boton usuario)
	GPIOA -> MODER |= (11 << 0*2);			//Configuracion PA0 como analogico
}

#endif /* INC_GPIO_CONF_H_ */
