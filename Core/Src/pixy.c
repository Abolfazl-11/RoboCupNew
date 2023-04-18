/*
 * pixy.c
 *
 *  Created on: Apr 18, 2023
 *      Author: dalvi
 */

#include "pixy.h"

// Version Request for pixy
uint8_t versionRequest[] = {
		0xae,
		0xc1,
		0x0e,
		0x00
};

// getBlocks Request for pixy
uint8_t getBlocks[] = {
		174,
		193,
		32,
		2,
		7,
		1
};

uint8_t buffer_rx[32];

void SetupPixy(int *pixyChecked) {
	while(HAL_SPI_Receive(&hspi1, buffer_rx, 1, 1000));

	HAL_SPI_Transmit(&hspi1, versionRequest, 4, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, buffer_rx, 8, 1000);
	HAL_SPI_Receive(&hspi1, buffer_rx, 22, 1000);

	*pixyChecked = 1;
}

void getBallPosition(BallTransform *ball_transform, int *ballInView) {

	// sending the getBlocks request and receiving datas
	// from pixy
	HAL_SPI_Transmit(&hspi1, getBlocks, 6, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, buffer_rx, 8, 1000); //garbage values
	HAL_SPI_Receive(&hspi1, buffer_rx, 4, 1000);

	// checking if the length received data is 14 and the
	// ball is in the view and return from the function if it's not
	if (buffer_rx[3] != 14) {
		ballInView = 0;
		return;
	}

	// if the length of the data is 14 will receive the 14 remaining bytes
	HAL_SPI_Receive(&hspi1, buffer_rx, 14, 1000);

	// saving the received data into the stuct
	ball_transform->ballx = buffer_rx[4] + buffer_rx[5] * 255;
	ball_transform->bally = buffer_rx[6] + buffer_rx[7] * 255;
	ball_transform->ballWidth = buffer_rx[8] + buffer_rx[9] * 255;
	ball_transform->ballHeight = buffer_rx[10] + buffer_rx[11] * 255;

	// cropping the received ball position to be only in the mirror
	if (!(ball_transform->ballx > PIXY_X_MIN && ball_transform->ballx < PIXY_X_MAX)) {
		*ballInView = 0;
		return;
	}
	if (!(ball_transform->bally > PIXY_Y_MIN && ball_transform->bally < PIXY_Y_MAX)) {
		*ballInView = 0;
		return;
	}

	*ballInView = 1;

	// changing the center of the image from the top left corner to the center of mirror
	if (ball_transform->ballx >= PIXY_X_ZERO) {
		ball_transform->ballx -= PIXY_X_ZERO;
	}else {
		ball_transform->ballx = -1 * (PIXY_X_ZERO - ball_transform->ballx);
	}

	if (ball_transform->bally >= PIXY_Y_ZERO) {
		ball_transform->bally = -1 * (PIXY_Y_ZERO - ball_transform->bally);
	}else {
		ball_transform->bally -= PIXY_Y_ZERO;
	}

	ball_transform->ballx *= -1;
	ball_transform->bally *= -1;

	for (int i = 0; i < 26; i++) {
		buffer_rx[i] = 0;
	}
}
