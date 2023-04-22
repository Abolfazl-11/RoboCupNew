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

    uint32_t t = 0;
    while(HAL_GPIO_ReadPin(sr->echo_port, sr->echo_pin) == GPIO_PIN_SET) {
    	t++;
    	delay_us(1);

    	if (t > 3000) break;
    }

    switch(sr->num) {
    case 0:
    	Datas->SR_f = (double)(t*340)/2000;
    	break;
    case 1:
    	Datas->SR_b = (double)(t*340)/2000;
    	break;
    case 2:
    	Datas->SR_l = (double)(t*340)/2000;
    	break;
    case 3:
    	Datas->SR_r = (double)(t*340)/2000;
    	break;
    }
}


void ReadAllSRs(SRDef_t *Srs[4], int c, SRDatas_t *Datas) {
	for(int i = 0; i < c; i++) {
		ReadSR(Srs[i], Datas);
	}
}
