/*
 * pixy.h
 *
 *  Created on: Apr 17, 2023
 *      Author: dalvi
 */
#include <spi.h>
#include "main.h"

#ifndef INC_PIXY_H_
#define INC_PIXY_H_

#define PIXY_Y_ZERO 122
#define PIXY_X_ZERO 158
#define PIXY_X_MIN 83
#define PIXY_X_MAX 230
#define PIXY_Y_MIN 51
#define PIXY_Y_MAX 197


// output struct of pixy file
typedef struct BallTransform {
	int ballx;
	int bally;
	int ballHeight;
	int ballWidth;
} BallTransform;

// this function sends the version request
// to pixy with SPI to setup pixy
void SetupPixy(int *pixyChecked);

// this function sends the get blocks request to pixy
// and receive the data from pixy and store the corrected
// values in "the ball_transfrom" struct and if the pixy is
// seeing the ball it will chnage the value of the "ballInView" to 1
// and if it's not it will change it to 0
void getBallPosition(BallTransform *ball_transform, int *ballInView);

#endif /* INC_PIXY_H_ */
