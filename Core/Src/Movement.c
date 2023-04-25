/*
 * Movement.c
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "Movement.h"

void RotateToZero(double e, double *pve, Motors_t *Motors, Motor_Defs *MotorDefs) {
	// PID Speed
	uint32_t u = abs((int)(Kp * abs(e) + Ki * (abs(e) * TIME) + Kd * (abs(e) - abs(*pve))));

	if (u > MAXROTATESPEED) u = MAXROTATESPEED;

	uint32_t stmp = 0;
	int en = e > 0 ? 1 : 0;

	// adding PID speed each motor speed individually and storing the first speed in the "stmp"
	// and setting it to "Motors" struct so we don't loose the actual speed of each motor
	if(en) {

		// Setting Motor_1 speed
		if(u > Motors->pwm1) {
			stmp = Motors->pwm1;
			setPWM(MotorDefs->Motor_1, u - Motors->pwm1, !Motors->e1, Motors);
			Motors->pwm1 = stmp;
			Motors->e1 = !Motors->e1;
		}
		else if (u < Motors->pwm1) {
			stmp = Motors->pwm1;
			// for robot 0
			setPWM(MotorDefs->Motor_1, Motors->pwm1 - (pow(-1, Motors->e1) * u), Motors->e1, Motors);
			// for robot 1
//			setPWM(MotorDefs->Motor_1, Motors->pwm1 - (pow(-1, !Motors->e1) * u), Motors->e1, Motors);
			Motors->pwm1 = stmp;
		}

		// Setting Motor_2 speed
		if(u > Motors->pwm2) {
			stmp = Motors->pwm2;
			setPWM(MotorDefs->Motor_2, u - Motors->pwm2, !Motors->e2, Motors);
			Motors->pwm2 = stmp;
			Motors->e2 = !Motors->e2;
		}
		else if (u < Motors->pwm2) {
			stmp = Motors->pwm2;
			setPWM(MotorDefs->Motor_2, Motors->pwm2 - (pow(-1, !Motors->e2) * u), Motors->e2, Motors);
			Motors->pwm2 = stmp;
		}

		// Setting Motor_3 speed
		stmp = Motors->pwm3;
		// for robot 1
//		setPWM(MotorDefs->Motor_3, Motors->pwm3 + (pow(-1, Motors->e3) * u), Motors->e3, Motors);
		// for robot 0
		setPWM(MotorDefs->Motor_3, Motors->pwm3 + (pow(-1, !Motors->e3) * u), Motors->e3, Motors);
		Motors->pwm3 = stmp;

		// Setting Motor_4 speed
		stmp = Motors->pwm4;
		setPWM(MotorDefs->Motor_4, Motors->pwm4 + (pow(-1, Motors->e4) * u), Motors->e4, Motors);
		Motors->pwm4 = stmp;
	}
	if(!en) {
		// Setting Motor_3 speed
		if(u > Motors->pwm3) {
			stmp = Motors->pwm3;
			setPWM(MotorDefs->Motor_3, u - Motors->pwm3, !Motors->e3, Motors);
			Motors->pwm3 = stmp;
			Motors->e3 = !Motors->e3;
		}
		else if (u < Motors->pwm3) {
			stmp = Motors->pwm3;
			setPWM(MotorDefs->Motor_3, Motors->pwm1 - (pow(-1, Motors->e3) * u), Motors->e3, Motors);
			Motors->pwm3 = stmp;
		}

		// Setting Motor_4 speed
		if(u > Motors->pwm4) {
			stmp = Motors->pwm4;
			setPWM(MotorDefs->Motor_4, u - Motors->pwm4, !Motors->e4, Motors);
			Motors->pwm4 = stmp;
			Motors->e4 = !Motors->e4;
		}
		else if (u <= Motors->pwm4) {
			stmp = Motors->pwm4;
			setPWM(MotorDefs->Motor_4, Motors->pwm4 - (pow(-1, Motors->e4) * u), Motors->e4, Motors);
			Motors->pwm4 = stmp;
		}

		// Setting Motor_1 speed
		stmp = Motors->pwm1;
		// for robot 0
		setPWM(MotorDefs->Motor_1, Motors->pwm1 + (pow(-1, Motors->e1) * u), Motors->e1, Motors);
		// for robot 1
//		setPWM(MotorDefs->Motor_1, Motors->pwm1 + (pow(-1, !Motors->e1) * u), Motors->e1, Motors);
		Motors->pwm1 = stmp;

		// Setting Motor_2 speed
		stmp = Motors->pwm2;
		setPWM(MotorDefs->Motor_2, Motors->pwm2 + (pow(-1, !Motors->e2) * u), Motors->e2, Motors);
		Motors->pwm2 = stmp;
	}

	*pve = e;
}

// this function gets the X and Y coordinate of the ball and
// converts them into polar coordinates then checks and sets the current zone
// and sets the motors with "GotoPoint" function from the "Motors.h" header
void GetBall(int x, int y, uint32_t speed, enum Zones *zone, Motors_t *Motors, Motor_Defs *MotorDefs, int *GoalCheck, SRDatas_t *SRDatas) {
	double teta;
	double r;

	if (x >= 0) teta = -(atan((double)y / x) * RAD_TO_DEG - 90);
	else if (x < 0) teta = -((atan((double)y/ x) + PI)* RAD_TO_DEG - 90);
	r = sqrt(x * x + y * y);

	if (r >= ZONEDIS_TH) {
		*zone = FAR;
		*GoalCheck = 0;
	}
	else if (r < ZONEDIS_TH) {
		*GoalCheck = 0;
		if (abs(teta) > GETBALLANGLE_TH) {
			*zone = CLOSE;
		}
		else {
			*zone = BALLIN;
		}
	}
	else {
		*zone = NA;
	}

	switch (*zone) {
	case FAR:
		GotoPoint(teta, speed, Motors, MotorDefs);
		break;
	case CLOSE:
		if (teta >= 0) {
			GotoPoint(teta + GETBALLANGLE, speed, Motors, MotorDefs);
		}
		else {
			GotoPoint(teta - GETBALLANGLE, speed, Motors, MotorDefs);
		}
		break;
	case BALLIN:
		GotoPoint(0, speed + 5, Motors, MotorDefs);
		break;
	case NA:
		AllMotorsZero(MotorDefs, Motors);
		break;
	}
}

void BackToGoal(Motors_t *Motors, Motor_Defs *MotorDefs, int *GoalCheck,SRDatas_t *SRDatas) {
	if (abs(SRDatas->SR_l - SRDatas->SR_r) > ATTACKZONE_TH) {
		if (SRDatas->SR_l > SRDatas->SR_r) {
			GotoPoint(-170, 25, Motors, MotorDefs);
		}
		else {
			GotoPoint(170, 25, Motors, MotorDefs);
		}
	} else {
		GotoPoint(-180, 30, Motors, MotorDefs);
	}
	if (SRDatas->SR_b <= GOALDIS_TH) {
		*GoalCheck = 1;
		AllMotorsZero(MotorDefs, Motors);
	}
}

void Attack(int x, int y, Motors_t *Motors, Motor_Defs *MotorDefs, SRDatas_t *SRDatas, enum Zones *zone, enum AttackZones *attackZone, int speed) {
	double teta;
	double r;

	if (x >= 0) teta = -(atan((double)y / x) * RAD_TO_DEG - 90);
	else if (x < 0) teta = -((atan((double)y/ x) + PI)* RAD_TO_DEG - 90);
	r = sqrt(x * x + y * y);

	if (r >= ZONEDIS_TH) {
		*zone = FAR;
		return;
	}
	else if (r < ZONEDIS_TH) {
		if (abs(teta) > AGETBALLANGLE_TH) {
			*zone = CLOSE;
			return;
		}
		else {
			if (abs(SRDatas->SR_l - SRDatas->SR_r) > ATTACKZONE_TH) {
				if (SRDatas->SR_l > SRDatas->SR_r) {
					*attackZone = RIGHT;
				}
				else {
					*attackZone = LEFT;
				}
			} else {
				*attackZone = MIDDLE;
			}
		}
	}

	switch (*attackZone) {
	case MIDDLE:
		GotoPoint(0, speed, Motors, MotorDefs);
		break;
	case RIGHT:
		GotoPoint(-ATTACKANGLE, speed, Motors, MotorDefs);
		break;
	case LEFT:
		GotoPoint(ATTACKANGLE, speed, Motors, MotorDefs);
		break;
	}
}
