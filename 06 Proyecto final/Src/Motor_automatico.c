/*
 * AnchoPWM_automatico.c
 *
 *  Created on: 17 dic. 2022
 *      Authors: Esther and Sara
 */

#include <Motor_automatico.h>
#include "main.h"

extern RTC_TimeTypeDef sTime;
extern int arranque_manual;
int iteracion_inicial = 0;

float Marcha_motor(float *temperatura, float *humedad, float *lux){

	if (iteracion_inicial == 0){				//Setup primera iteracion modo automatico
		GPIOA -> BSRR = (1 << 6);				//Encender LED motor ON
		TIM2 -> CR1 |= (1 << 0);				//Arrancar timer 2 (PWM)
		TIM2 -> CCER |= (1 << 4);				//Habilitacion canal 2 (CC2E)
		GPIOB -> BSRR = (1 << 5);				//Habilitar driver motor

		iteracion_inicial = 1;
	}
	if ((sTime.Minutes % 2 != 0) && (20.0 > *temperatura && *temperatura >= 12.0) && (*humedad < 60.0) && (*lux < 30000)){
		TIM2 -> CCR2 = 5000 - 1;				//PWM 25% si la temperatura es baja y la humedad es alta
	}
	else if ((sTime.Minutes % 2 != 0) && ((20.0 > *temperatura && *temperatura >= 12.0) && (*humedad < 50.0) || (30.0 > *temperatura && *temperatura >= 20.0) && (*humedad < 60.0)) && (*lux < 30000)){
		TIM2 -> CCR2 = 10000 - 1;				//PWM 50% si la temperatura es baja y la humedad es baja o si la temperatura es media y la humedad es alta
	}
	else if ((sTime.Minutes % 2 != 0) && ((30.0 > *temperatura && *temperatura >= 20.0) && (*humedad < 50.0) || (30.0 >= *temperatura) && (*humedad < 60.0)) && (*lux < 30000)){
		TIM2 -> CCR2 = 15000 - 1;				//PWM 75% si la temperatura es alta y la humedad es media o si la temperatura es muy alta y la humedad es alta
	}
	else if ((sTime.Minutes % 2 != 0) && (30.0 >= *temperatura) && (*humedad < 50.0) && (*lux < 30000)){
		TIM2 -> CCR2 = 20001 - 1;				//PWM 100% si la temperatura es alta y la humedad es baja
	}
	else{
		Parar_motor();							//No alimentar la bomba si no se cumple ninguna de estas condiciones
	}
}

void Parar_motor(){
	iteracion_inicial = 0;

	GPIOA -> BSRR = (1 << 6) << 16;			//Apagar LED motor ON
	TIM2 -> CR1 &= ~(1 << 0);				//Apagar timer 2 (PWM)
	TIM2 -> CCER &= ~(1 << 4);				//Deshabilitacion canal 2 (CC2E)
	GPIOB -> BSRR = (1 << 5) << 16;			//Deshabilitar driver motor

	TIM3 -> CR1 &= ~(1 << 0);				//Deshabilitar timer 3 (regular velocidad PWM modo manual)
}
