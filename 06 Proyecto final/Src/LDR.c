/*
 * LDR.c
 *
 *  Created on: Dec 9, 2022
 *      Authors: Esther and Sara
 */

#include "main.h"
#include <LDR.h>

extern int valor_luminosidad;

static const float RLDR_10lux = 50000.0;	//Valor de la LDR a 10 lux (datasheet)
static const float constanteLDR = 0.7;		//Constante de la LDR (datasheet)

int Rc = 10000;								//Resistencia divisor de tension
float tension = 0.0;						//Tension a la entrada del ADC
float R_LDR = 0.0;							//Valor de la resistencia LDR
float R_Temp = 0.0;							//Valor R linealizada


float Medir_lux(float *lux){
	tension = valor_luminosidad * 3.3 / 4095.0;					//Calculo del valor de la tension LDR
	R_LDR = (Rc * tension)/(3.3 - tension);						//Calculo valor resistencia LDR
	R_Temp = 0.42*log(R_LDR/RLDR_10lux) / -constanteLDR + 1;	//Obtencion R linealizada
	*lux = pow(10,R_Temp);										//Obtencion lux
}
