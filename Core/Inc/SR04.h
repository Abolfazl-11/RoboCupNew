/*
 * SR04.h
 *
 *  Created on: Apr 20, 2023
 *      Author: dalvi
 */

#include "main.h"
#include "tim.h"

#ifndef INC_SR04_H_
#define INC_SR04_H_

#define FIELDWIDTHSUM 120
#define FIELDLENGTHSUM 300

typedef struct SRDef_t {
	uint16_t trig_pin;
	GPIO_TypeDef *trig_port;
	uint16_t echo_pin;
	GPIO_TypeDef *echo_port;
	int num;
} SRDef_t;

typedef struct SRDatas_t {
	double SR_f;
	double SR_b;
	double SR_l;
	double SR_r;
} SRDatas_t;

// delay in microseconds
void delay_us(int us);

// function to read SRs
void ReadSR(SRDef_t *SR, SRDatas_t *Datas);

void ReadAllSRs(SRDef_t *SRs[4], int c, SRDatas_t *Datas);

#endif /* INC_SR04_H_ */
