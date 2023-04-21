/*
 * Motors.c
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "Motors.h"

void setPWM(MotorDef_t *Motor, uint32_t pwm, int en, Motors_t *Motors) {
	switch(Motor->timer) {
		case(1):
			switch(Motor->channel){
				case(2):
						TIM1->CCR2 = en * (100 - (pwm *1.1)) + !en * pwm;
						Motors->pwm1 = pwm;
						Motors->e1 = en;
						break;
				case(3):
						TIM1->CCR3 = en * (100 - (pwm * 1.1)) + !en * pwm;
						Motors->pwm2 = pwm;
						Motors->e2 = en;
						break;
				case(4):
						TIM1->CCR4 = en * (100 - pwm) + !en * pwm;
						Motors->pwm4 = pwm;
						Motors->e4 = en;
						break;
			}
			break;
		case(2):
			switch(Motor->channel) {
				case(4):
					TIM2->CCR4 = en * (100 - pwm) + !en * pwm;
					Motors->pwm3 = pwm;
					Motors->e3 = en;
					break;
			}
			break;
	}
	HAL_GPIO_WritePin(GPIOA, Motor->in1, en);
}

// offset angle of each motor
int offsets[] = {-45, -135, 135, 45};

void GotoPoint(double teta, uint32_t speed, Motors_t *Motors, Motor_Defs *MotorDefs) {
	for (int i = 0; i < 4; ++i) {
		double t = teta + offsets[i];
		double s = sin(t * DEG_TO_RAD) * speed;
		if (s > MAXSPEED) s = MAXSPEED;
		int en = s >= 0 ? 1 : 0;
		s = abs(s);
		if (en == 0) {
			switch(i) {
			case 0:
				setPWM(MotorDefs->Motor_1, s, en, Motors);
				break;
			case 1:
				setPWM(MotorDefs->Motor_2, s, !en, Motors);
				break;
			case 2:
				setPWM(MotorDefs->Motor_3, s, en, Motors);
				break;
			case 3:
				setPWM(MotorDefs->Motor_4, s, !en, Motors);
				break;
			}
		}
		else if (en == 1) {
			switch(i) {
				case 0:
					setPWM(MotorDefs->Motor_1, s, en, Motors);
					break;
				case 1:
					setPWM(MotorDefs->Motor_2, s, !en, Motors);
					break;
				case 2:
					setPWM(MotorDefs->Motor_3, s, en, Motors);
					break;
				case 3:
					setPWM(MotorDefs->Motor_4, s, !en, Motors);
					break;
			}
		}
	}

//	teta -= 135;
//	int s = sin(teta * DEG_TO_RAD) * speed;
//	int en = s > 0 ? 1 : 0;
//	s = abs(s);
//	s = s > MAXSPEED ? MAXSPEED : s;
//	setPWM(&Motor_2, s, en, Motors);
//	setPWM(&Motor_4, s, !en, Motors);
//	s = cos(teta * DEG_TO_RAD) * speed;
//	en = s > 0 ? 1 : 0;
//	s = abs(s);
//	s = s > MAXSPEED ? MAXSPEED : s;
//	setPWM(&Motor_1, s, en, Motors);
//	setPWM(&Motor_3, s, en, Motors);
}

void AllMotorsZero(Motor_Defs *MotorDefs, Motors_t *Motors) {
	// For robot 1
	setPWM(MotorDefs->Motor_1, 0, 0, Motors);
	setPWM(MotorDefs->Motor_2, 0, 1, Motors);
	setPWM(MotorDefs->Motor_3, 0, 1, Motors);
	setPWM(MotorDefs->Motor_4, 0, 0, Motors);

	// For robot 0
//	setPWM(MotorDefs->Motor_1, 0, 0, Motors);
//	setPWM(MotorDefs->Motor_2, 0, 0, Motors);
//	setPWM(MotorDefs->Motor_3, 0, 0, Motors);
//	setPWM(MotorDefs->Motor_4, 0, 1, Motors);
}
