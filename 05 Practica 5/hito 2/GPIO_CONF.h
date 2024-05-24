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
	RCC -> AHB1ENR |= (1 << 0);				//Habilitacion reloj puerto A

	//CONFIGURACION ENTRADAS/SALIDAS
	GPIOA -> MODER |= (10 << (1*2));		//Configuracion funcion alternativa TIM2_CH2
	GPIOA -> AFR[0] |= (0001 << 1*4);		//Funcion alternativa 1 para PA1 (TIM2_CH2)
}

#endif /* INC_GPIO_CONF_H_ */
