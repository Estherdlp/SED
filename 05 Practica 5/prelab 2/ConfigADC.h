/*
 * ConfigADC.h
 *
 *  Created on: 5 dic. 2022
 *      Author: Esther
 */

//Configuracion ADC modo simple puerto PA0

#ifndef INC_CONFIGADC_H_
#define INC_CONFIGADC_H_

void ConfigADC(){							//Parametros ADC
	RCC -> APB2ENR |= (1 << 8);				//Habilitacion reloj ADC1

	ADC -> CCR |= (1 << 16);				//Frecuencia < 36 MHz

	ADC1 -> CR2 &= ~(1 << 0); 				//Apagar ADC para realizar la configuracion
	ADC1 -> CR2 &= ~(1 << 1); 				//Bit CONT = 0 para conversion simple
	ADC1 -> CR2 |= (1 << 10); 				//Flag end of conversion despues de cada conversion
	ADC1 -> CR2 &= ~(1 << 11); 				//Big align a 0 para alinear los bits a la derecha
	ADC1 -> CR2 |= (1 << 28);				//External trigger enable = 1
	ADC1 -> CR2 |= (1 << 27);				//TIM3_TRGO event

	ADC1 -> CR1 |= (1 << 5); 				//Habilitada interrupcion fin conversion, resolucion de 12 bits, un solo canal, deshabilitacion de interrupcion por desbordamiento

	ADC1 -> SQR1 = 0;						//Secuencia de un elemento
	ADC1 -> SQR3 &= ~(11111 < 0);			//Canal PA0

	ADC1 -> SMPR2 &= ~(111 << 3);			//Muestrear en 3 ciclos

	ADC1 -> CR2 |= (1 << 0);				//Encender ADC una vez configurado
}

#endif /* INC_CONFIGADC_H_ */
