/*
 * blackbox.h
 *
 *      Author: Nicholas Sherlock
 */

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include <stdint.h>

typedef struct blackbox_values_t {
	uint32_t time;

	int32_t axisP[3], axisI[3], axisD[3];

	int16_t rcCommand[4];
	int16_t gyroData[3];
	int16_t accSmooth[3];
	int16_t motor[8];
} blackbox_values_t;

void initBlackbox(void);
void handleBlackbox(void);
void startBlackbox(void);
void finishBlackbox(void);

#endif /* BLACKBOX_H_ */
