#include "tools.h"

double doubleAbs(double a)
{
	if (a < 0)
		return -a;
	return a;
}

double doubleMin(double a, double b)
{
	if (a < b)
		return a;
	return b;
}

double doubleMax(double a, double b)
{
	if (a > b)
		return a;
	return b;
}

float intToFloat(int32_t i)
{
	floatConvert_t convert;

	convert.i = i;
	return convert.f;
}

float uintToFloat(uint32_t u)
{
	floatConvert_t convert;

	convert.u = u;
	return convert.f;
}

int32_t floatToInt(float f)
{
	floatConvert_t convert;

	convert.f = f;
	return convert.i;
}

uint32_t floatToUint(float f)
{
	floatConvert_t convert;

	convert.f = f;
	return convert.u;
}
