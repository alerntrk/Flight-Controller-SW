/*
 * receiver.c
 *
 *  Created on: Mar 14, 2023
 *      Author: AEren TURK
 */
#include "types.h"
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0;
volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0;
volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0;
extern struct ReceiverCommands _ReceiverCommands;

float newch1,newch2,newch4;

void mapAngleP(){

	newch2=(float)(ch2-TX_MIN)*(MAX_TILT_ANGLE-MIN_TILT_ANGLE)/(TX_MAX-TX_MIN)+MIN_TILT_ANGLE;

}
void mapAngleR(){

	newch1=(float)(ch1-TX_MIN)*(MAX_TILT_ANGLE-MIN_TILT_ANGLE)/(TX_MAX-TX_MIN)+MIN_TILT_ANGLE;

}
void mapThrottle(){

	newch4=(float)(ch3-TX_MIN)*(THROTTLE_MAX-1000)/(TX_MAX-TX_MIN)+1000;

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)  // interrupt TIM2 biriminden geliyorsa gir
	{
		switch(htim->Channel) // aktif kanal hamgisiyse, o kanalın case'ine git
		{


		case HAL_TIM_ACTIVE_CHANNEL_1:
			if((TIM2->CCER & TIM_CCER_CC1P)==0)
			{
				ch1_rising = TIM2->CCR1;
				TIM2->CCER |= TIM_CCER_CC1P;
			}
			else
			{
				ch1_falling = TIM2->CCR1;
				pre_ch1 = ch1_falling - ch1_rising;
				if(pre_ch1 < 0)pre_ch1 += 0xFFFF;
				if(pre_ch1 < 2000 && pre_ch1 > 1000){
					ch1=pre_ch1;
					mapAngleR();

				}
				TIM2->CCER &= ~TIM_CCER_CC1P;
			}
			break;

		case HAL_TIM_ACTIVE_CHANNEL_2:
			if((TIM2->CCER & TIM_CCER_CC2P)==0)
			{
				ch2_rising = TIM2->CCR2;
				TIM2->CCER |= TIM_CCER_CC2P;
			}
			else
			{
				ch2_falling = TIM2->CCR2;
				pre_ch2 = ch2_falling - ch2_rising;
				if(pre_ch2 < 0)pre_ch2 += 0xFFFF;
				if(pre_ch2 < 2000 &&  pre_ch2 > 1000){
					ch2=pre_ch2;
					mapAngleP();
				}
				TIM2->CCER &= ~TIM_CCER_CC2P;

			}
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			if((TIM2->CCER & TIM_CCER_CC3P)==0)
			{
				ch3_rising = TIM2->CCR3;
				TIM2->CCER |= TIM_CCER_CC3P;
			}
			else
			{
				ch3_falling = TIM2->CCR3;
				pre_ch3 = ch3_falling - ch3_rising;
				if(pre_ch3 < 0)pre_ch3 += 0xFFFF;
				if(pre_ch3 < 2000 && pre_ch3 > 1000){
					ch3=pre_ch3;
					mapThrottle();

				}
				TIM2->CCER &= ~TIM_CCER_CC3P;
			}
			break;

		default:
			break;
		}
	}
	if(htim->Instance == TIM3)  // interrupt TIM2 biriminden geliyorsa gir
		{

				if((TIM3->CCER & TIM_CCER_CC4P)==0) // kanalin aktif olmasi kesmenin oradan gelecegi anlamina gelmez/gpio pinini kontrol et
				{
					ch1_rising = TIM3->CCR4; // yukselen kenar degerini kaydet
					TIM3->CCER |= TIM_CCER_CC4P; // polariteyi düsen kenar olarak degistir
				}

				else
				{
					ch1_falling = TIM3->CCR4;
					pre_ch1 = ch1_falling - ch1_rising; // dusen kenar degerini kaydet ve yukselen kenar degerinden cikar
					if(pre_ch1 < 0)pre_ch1 += 0xFFFF;// eger sonuc negatifse taban tumleme yap
					if(pre_ch1 < 2000 && pre_ch1 > 1000){
						ch1=pre_ch1;
						mapAngleR();
					}
					TIM3->CCER &= ~TIM_CCER_CC4P; // polariteyi yukselen kenar olarak ayarla

				}




}

	_ReceiverCommands.Throttle=newch4;
	_ReceiverCommands.PitchAngle=newch2;
	_ReceiverCommands.RollAngle=newch1;
}
void Receiver_Init(){
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	  //HAL_TIM_IC_Start_IT(&hTIM2, TIM_CHANNEL_4);

}


