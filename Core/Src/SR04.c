/*
 * SR04.c
 *
 *  Created on: Apr 20, 2023
 *      Author: dalvi
 */

#include <SR04.h>

void delay_us(int us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

void ReadSR(SRDef_t *sr, SRDatas_t *Datas) {
	HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_SET);
	delay_us(10);
    HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_RESET);

    while(HAL_GPIO_ReadPin(sr->echo_port, sr->echo_pin) != GPIO_PIN_SET);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while(HAL_GPIO_ReadPin(sr->echo_port, sr->echo_pin) == GPIO_PIN_SET) {
   	if(__HAL_TIM_GET_COUNTER(&htim4) > 3000) break;
    }

    switch(sr->num) {
    case 0:
    	Datas->SR_f = (double)(__HAL_TIM_GET_COUNTER(&htim4)*340)/2000;
    	break;
    case 1:
    	Datas->SR_b = (double)(__HAL_TIM_GET_COUNTER(&htim4)*340)/2000;
    	break;
    case 2:
    	Datas->SR_l = (double)(__HAL_TIM_GET_COUNTER(&htim4)*340)/2000;
    	break;
    case 3:
    	Datas->SR_r = (double)(__HAL_TIM_GET_COUNTER(&htim4)*340)/2000;
    	break;
    }
}


void ReadAllSRs(SRDef_t *Srs[4], int c, SRDatas_t *Datas) {
	for(int i = 0; i < c; i++) {
		ReadSR(Srs[i], Datas);
	}
}
