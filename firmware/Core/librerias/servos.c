/*
 * servos.c
 *
 *  Created on: Apr 8, 2023
 *      Author: Alcides Ramos
 */


#include "servos.h"


//incluir los timer usados
extern TIM_HandleTypeDef htim2; // para los alerones
extern TIM_HandleTypeDef htim1; // para el paracaídas

//Configurar Servos Usados
SERVOS SERVO0 = {&htim1, &(TIM1->CCR4),TIM_CHANNEL_4}; // paracaídas, le modifiqué el canal a usar del timer
SERVOS SERVO1 = {&htim2, &(TIM2->CCR1),TIM_CHANNEL_1}; // alerón 1
SERVOS SERVO2 = {&htim2, &(TIM2->CCR2),TIM_CHANNEL_2}; // alerón 2
SERVOS SERVO3 = {&htim2, &(TIM2->CCR3),TIM_CHANNEL_3}; // alerón 3
SERVOS SERVO4 = {&htim2, &(TIM2->CCR4),TIM_CHANNEL_4}; // alerón 4

void   SERVO_init(SERVOS *servo)
{
HAL_TIM_PWM_Start(servo->htim, servo->channel);

}


void SERVO_ANG(SERVOS *servo,float posi)

{
     float calcu;
      calcu=(ser_lim_sup_ms - ser_lim_inf_ms) / (ser_sup - ser_inf);
	  calcu=calcu*(posi-ser_inf);
	  calcu=(calcu+ser_lim_inf_ms)*1000.0;// para pasarlo a microsegundo
	  *servo->ccr = calcu;

}


void SERVO_MICRO(SERVOS *servo,float micro)

{
     *servo->ccr = micro;
}


void SERVO_MILI(SERVOS *servo,float milis)

{
     float calcu;
      calcu=milis*1000.0;// para pasarlo a microsegundo
	  *servo->ccr = calcu;
}
