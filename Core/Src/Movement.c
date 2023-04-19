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

	uint32_t stmp = 0;
	int en = e > 0 ? 1 : 0;

	// adding PID speed each motor speed individually and storing the first speed in the "stmp"
	// and setting it to "Motors" struct so we don't loose the actual speed of each motor
	if(en) {
		if(u > Motors->pwm1) {
			stmp = Motors->pwm1;
			setPWM(MotorDefs->Motor_1, u - Motors->pwm1, !Motors->e1, Motors);
			Motors->pwm1 = stmp;
			Motors->e1 = !Motors->e1;
		}
		else if (u < Motors->pwm1) {
			stmp = Motors->pwm1;
			setPWM(MotorDefs->Motor_1, Motors->pwm1 - u, Motors->e1, Motors);
			Motors->pwm1 = stmp;
		}
		if(u > Motors->pwm1) {
			stmp = Motors->pwm2;
			setPWM(MotorDefs->Motor_2, u - Motors->pwm2, !Motors->e2, Motors);
			Motors->pwm2 = stmp;
			Motors->e2 = !Motors->e2;
		}
		else if (u < Motors->pwm2) {
			stmp = Motors->pwm2;
			setPWM(MotorDefs->Motor_2, Motors->pwm2 - u, Motors->e2, Motors);
			Motors->pwm2 = stmp;
		}
		stmp = Motors->pwm3;
		setPWM(MotorDefs->Motor_3, Motors->pwm3 + u, Motors->e3, Motors);
		Motors->pwm3 = stmp;

		stmp = Motors->pwm4;
		setPWM(MotorDefs->Motor_4, Motors->pwm4 + u, Motors->e4, Motors);
		Motors->pwm4 = stmp;
	}
	if(!en) {
		if(u > Motors->pwm3) {
			stmp = Motors->pwm3;
			setPWM(MotorDefs->Motor_3, u - Motors->pwm3, !Motors->e3, Motors);
			Motors->pwm3 = stmp;
			Motors->e3 = !Motors->e3;
		}
		else if (u < Motors->pwm3) {
			stmp = Motors->pwm3;
			setPWM(MotorDefs->Motor_3, Motors->pwm1 - u, Motors->e3, Motors);
			Motors->pwm3 = stmp;
		}
		if(u > Motors->pwm4) {
			stmp = Motors->pwm4;
			setPWM(MotorDefs->Motor_4, u - Motors->pwm4, !Motors->e4, Motors);
			Motors->pwm4 = stmp;
			Motors->e4 = !Motors->e4;
		}
		else if (u <= Motors->pwm4) {
			stmp = Motors->pwm4;
			setPWM(MotorDefs->Motor_4, Motors->pwm4 - u, Motors->e4, Motors);
			Motors->pwm4 = stmp;
		}
		stmp = Motors->pwm1;
		setPWM(MotorDefs->Motor_1, Motors->pwm1 + u, Motors->e1, Motors);
		Motors->pwm1 = stmp;

		stmp = Motors->pwm2;
		setPWM(MotorDefs->Motor_2, Motors->pwm2 + u, Motors->e2, Motors);
		Motors->pwm2 = stmp;
	}

	*pve = e;
}

// this function gets the X and Y coordinate of the ball and
// converts them into polar coordinates then checks and sets the current zone
// and sets the motors with "GotoPoint" function from the "Motors.h" header
void GetBall(int x, int y, uint32_t speed, enum Zones *zone, Motors_t *Motors, Motor_Defs *MotorDefs) {
	double teta;
	double r;

	if (x >= 0) teta = -(atan((double)y / x) * RAD_TO_DEG - 90);
	else if (x < 0) teta = -((atan((double)y/ x) + PI)* RAD_TO_DEG - 90);

	r = sqrt(x * x + y * y);

	if (r >= ZONEDIS_TH) {
		*zone = FAR;
	}
	else if (r < ZONEDIS_TH) {
		if (abs(teta) > GETBALLANGLE_TH) {
			*zone = CLOSE;
		}
		else {
			*zone = BALLIN;
		}
	}

	switch (*zone) {
	case FAR:
		GotoPoint(teta, speed, Motors, MotorDefs);
		break;
	case CLOSE:
		if (teta >= 0) {
			GotoPoint(teta + 30, speed, Motors, MotorDefs);
		}
		else {
			GotoPoint(teta - 30, speed, Motors, MotorDefs);
		}
		break;
	case BALLIN:
		GotoPoint(0, speed + 5, Motors, MotorDefs);
	}
}
