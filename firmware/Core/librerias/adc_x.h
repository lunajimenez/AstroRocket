/*
 * adc.h
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */

#ifndef LIBRERIAS_ADC_H_
#define LIBRERIAS_ADC_H_
//configurar el DMA en normal si se usa dma , mas de un canal

#include "main.h"
//habilite los adc a usar
extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;


#define adc_canales  1 //  coloque numuro de canales del ADC a usar

extern uint16_t adc_codigo[];


uint16_t ADC_Read(ADC_HandleTypeDef  *adc_n);
void ADC_Read_DMA(ADC_HandleTypeDef  *adc_n);



#endif /* LIBRERIAS_ADC_H_ */


