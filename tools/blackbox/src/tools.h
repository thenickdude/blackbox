#ifndef TOOLS_H_
#define TOOLS_H_

#include <stdint.h>

typedef union floatConvert_t {
	float f;
	uint32_t u;
	int32_t i;
} floatConvert_t;

float intToFloat(int32_t i);
float uintToFloat(uint32_t u);
int32_t floatToInt(float f);
uint32_t floatToUint(float f);

double doubleAbs(double a);
double doubleMin(double a, double b);
double doubleMax(double a, double b);

#endif
