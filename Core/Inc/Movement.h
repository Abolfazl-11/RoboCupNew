/*
 * Movement.h
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "Motors.h"
#include "main.h"
#include "math.h"
#include "SR04.h"

#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_

#define Kp 2.1
#define Ki 1.5
#define Kd 1.6

#define TIME 0.000048
#define MAXROTATESPEED 20

#define ZONEDIS_TH 50
#define GETBALLANGLE_TH 11
#define ATTACKZONE_TH 20

#define GOALDIS_TH 70

// this enum indicates the current zone of the robot
// related to the ball position
// FAR : the ball is farther than the "ZONEDIS_TH"
// CLOSE : the ball is Closer than the "ZONEDIS_TH"
// BALLIN : the ball closer than the "ZONEDIS_TH" and is in the get ball angle limit
enum Zones { NA, FAR, CLOSE, BALLIN };

enum AttackZones { LEFT, MIDDLE, RIGHT };

// The PID Control function to rotate the robot to 0 degrees and
// make robot always facing towards the enemy side
void RotateToZero(double e, double *pve, Motors_t *Motors, Motor_Defs *MotorDefs);

void GetBall(int x, int y, uint32_t speed, enum Zones *zone, Motors_t *Motors, Motor_Defs *MotorDefs, int *GoalCheck, SRDatas_t *SRDatas);

// this function brings back the keeper robot to the front of the Goal
// this function gets called when the robot isn't seeing the ball and isn't
// in front of the
void BackToGoal(Motors_t *Motors, Motor_Defs *MotorDefs, int *GoalCheck, SRDatas_t *SRDatas);

// This function is called when the robot gets the ball and is ready to
// attack and score a GOAL!
void Attack(Motors_t *Motors, Motor_Defs *MotorDefs, SRDatas_t *SRDatas, enum AttackZones *attackZone, int speed);

#endif /* INC_MOVEMENT_H_ */
