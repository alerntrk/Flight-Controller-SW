/*
 * Motor.c
 *
 *  Created on: 15 Mar 2023
 *      Author: AEren TÜRK
 */

#include "main.h"
#include "types.h"
extern TIM_HandleTypeDef htim1;
extern  MotorPowers motorPowers;


void Motor_Init(){

 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);


}
void Spin_Motor(){

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,motorPowers.frontLeftMotorPower);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motorPowers.frontRightMotorPower);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motorPowers.rearLeftMotorPower);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motorPowers.rearRightMotorPower);

}
void Stop_Motor(){

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);

}
