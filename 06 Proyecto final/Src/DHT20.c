/*
 * DHT20.c
 *
 *  Created on: 9 dic. 2022
 *      Authors: Esther and Sara
 */
#include "stdio.h"							//Librerias para imprimir por pantalla
#include "string.h"
#include "main.h"
#include <DHT20.h>


extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern char msg_res[500];					//Cadena mensaje

static const uint8_t DHT20_ADDR = (0x38 << 1); 		//Direccion del dispositivo + orden escritura (ultimo bit 0)
static const uint8_t DHT20_ADDR_LECTURA = 0x71; 	//Direccion del dispositivo + orden de lectura (ultimo bit 1)
static const uint8_t TRIGGER_MEDIDA[3] = {0xAC, 0x33, 0x00}; //Enviar peticion medida

HAL_StatusTypeDef estado;					//Flag comunicacion sensor DHT20

uint8_t buf[7];								//Buffer mediciones sensor
uint32_t raw_temperatura = 0;				//Valor raw temperatura
uint32_t raw_humedad = 0;					//Valor raw humedad


float Medir_Temperatura(float *temperatura){
	estado = HAL_I2C_Master_Transmit(&hi2c1, DHT20_ADDR, (uint8_t *) TRIGGER_MEDIDA, 3, HAL_MAX_DELAY);	//Orden de falsa escritura para iniciar la medicion
	HAL_Delay(80);																						//Esperar 80 ms antes de consultar la medida (indicado por fabricante)

	if (estado != HAL_OK) {																				//Fallo si el sensor no esta disponible (no ACK)
		sprintf(msg_res, "\r\nEl sensor no se encuentra disponible.");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
	}

	else {
	  estado = HAL_I2C_Master_Receive(&hi2c1, DHT20_ADDR_LECTURA, (uint8_t *) buf , 7, HAL_MAX_DELAY);  //Orden de lectura de los valores medidos

	  if (estado != HAL_OK) {																			//Fallo si el sensor no esta disponible (no ACK)
		  sprintf(msg_res, "\r\nError al recibir la informacion");
		  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
	  }

	  else {
		  raw_temperatura = (buf[5] & 0xFF) | ((buf[4] & 0xFF) << 8) | ((buf[3] & 0x0F) << 16);			//Rotacion de los bits para obtener lectura de temperatura
		  *temperatura = (raw_temperatura / (1024*1024.0)) * 200 - 50;									//Calculo de temperatura conforme ec. del fabricante

		  raw_temperatura = 0;
	  }
  }
}

float Medir_Humedad(float *humedad){
	estado = HAL_I2C_Master_Transmit(&hi2c1, DHT20_ADDR, (uint8_t *) TRIGGER_MEDIDA, 3, HAL_MAX_DELAY); //Orden de falsa escritura para iniciar la medicion
	HAL_Delay(80);																						//Esperar 80 ms antes de consultar la medida (indicado por fabricante)

	if (estado != HAL_OK) {																				//Fallo si el sensor no esta disponible (no ACK)
		sprintf(msg_res, "\r\nEl sensor no se encuentra disponible.");
		HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
	}

	else {
	  estado = HAL_I2C_Master_Receive(&hi2c1, DHT20_ADDR_LECTURA, (uint8_t *) buf , 7, HAL_MAX_DELAY);	//Orden de lectura de los valores medidos

	  if (estado != HAL_OK) {																			//Fallo si el sensor no esta disponible (no ACK)
		  sprintf(msg_res, "\r\nError al recibir la informacion");
		  HAL_UART_Transmit(&huart2, (char *) msg_res, strlen(msg_res), 100);
	  }

	  else {
		  raw_humedad = (buf[3] & 0xF0) | ((buf[2] & 0xFF) << 4) | ((buf[1] & 0xFF) << 12);				//Rotacion de los bits para obtener lectura de humedad
		  *humedad = (raw_humedad / (1024*1024.0)) * 100;												//Calculo de humedad conforme ec. del fabricante

		  raw_humedad = 0;
	  }
	}
}

