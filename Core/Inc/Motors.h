/*
 * Motors.h
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "main.h"
#include "tim.h"
#include "math.h"
#include <stdlib.h>

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

// max speed of motors will be this constant
#define MAXSPEED 30

#define MAXROTATESPEED 20

#define PI 3.14159265
#define DEG_TO_RAD PI/180
#define RAD_TO_DEG 180/PI

// this struct contains the PWM speed and
// enable pin state for each motor
typedef struct Motors_t {
	uint32_t pwm1;
	int e1;
	uint32_t pwm2;
	int e2;
	uint32_t pwm3;
	int e3;
	uint32_t pwm4;
	int e4;
} Motors_t;

// a struct to define motors with their timer and channel and enable pin
typedef struct MotorDef_t {
	uint16_t timer;
	uint16_t channel;
	uint16_t in1;
	GPIO_TypeDef *in1_port;
} MotorDef_t;

typedef struct MotorsDefs {
	MotorDef_t *Motor_1;
	MotorDef_t *Motor_2;
	MotorDef_t *Motor_3;
	MotorDef_t *Motor_4;
} Motor_Defs;
// this function sets the enable pin and PWM speed for motors
// it gets the motor  definition struct of the motor and the PWM speed
// an the enable state and set them to the motor and store the values in the "Motors" struct
void setPWM(MotorDef_t *Motor, uint32_t pwm, int en, Motors_t *Motors);

// this function gets the teta and speed and set the speed of each motor
// in a way that the robot will move in the angle of inputed teta
void GotoPoint(double teta, uint32_t speed, Motors_t *Motors, Motor_Defs *MotorDefs);

// this function sets the speed of all motors zero and stops the robot
void AllMotorsZero(Motor_Defs *MotorDefs, Motors_t *Motors);
#endif /* INC_MOTORS_H_ */
