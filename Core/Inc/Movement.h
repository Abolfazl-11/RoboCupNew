/*
 * Movement.h
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "Motors.h"
#include "main.h"
#include "math.h"

#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_

#define Kp 1.9
#define Ki 1.5
#define Kd 1.6

#define TIME 0.000048
#define MAXROTATESPEED 20

#define ZONEDIS_TH 50
#define GETBALLANGLE_TH 12

// this enum indicates the current zone of the robot
// related to the ball position
// FAR : the ball is farther than the "ZONEDIS_TH"
// CLOSE : the ball is Closer than the "ZONEDIS_TH"
// BALLIN : the ball is closer than the "ZONEDIS_TH" and is in the get ball angle limit
enum Zones { FAR, CLOSE, BALLIN };

// The PID Control function to rotate the robot to 0 degrees and
// make robot always facing towards the enemy side
void RotateToZero(double e, double *pve, Motors_t *Motors, Motor_Defs *MotorDefs);

void GetBall(int x, int y, uint32_t speed, enum Zones *zone, Motors_t *Motors, Motor_Defs *MotorDefs);
void BackToGoal();
void Attack();

#endif /* INC_MOVEMENT_H_ */
