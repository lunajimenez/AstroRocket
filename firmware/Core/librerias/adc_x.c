/*
 * adc.c
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */


#include "adc_x.h"


uint16_t adc_codigo[ adc_canales];

uint16_t ADC_Read(ADC_HandleTypeDef  *adc_n)
{
	HAL_ADC_Start(adc_n);
	HAL_ADC_PollForConversion(adc_n,1000);
	HAL_ADC_Stop(adc_n);
	return (HAL_ADC_GetValue(adc_n));
}

void ADC_Read_DMA(ADC_HandleTypeDef  *adc_n)
{
HAL_ADC_Start_DMA(adc_n, (uint32_t*)adc_codigo, adc_canales);
   HAL_Delay(1);//retardo de conversion si necesita
}

