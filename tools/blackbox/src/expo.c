#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "expo.h"

struct expoCurve_t {
	double *curve;
	double inputRange;
	int steps;
	double stepSize;
};

double expoCurveLookup(expoCurve_t *curve, double input)
{
	double normalisedInput, valueInCurve;
	int prevStepIndex;

	normalisedInput = input / curve->inputRange;

	valueInCurve = fabs(normalisedInput) * curve->steps;
	prevStepIndex = (int) valueInCurve;

	/* If the input value lies beyond the stated input range, use the final
	 * two points of the curve to extrapolate out (the "curve" out there is a straight line, though)
	 */
	if (prevStepIndex > curve->steps - 2) {
		prevStepIndex = curve->steps - 2;
	}

	//Straight-line interpolation between the two curve points
	double proportion = (normalisedInput - curve->stepSize * prevStepIndex) / curve->stepSize;
	double result = curve->curve[prevStepIndex] * (1 - proportion) + curve->curve[prevStepIndex + 1] * proportion;

	if (input < 0)
		return -result;
	return result;
}

expoCurve_t *expoCurveCreate(double power, double inputRange, double outputRange, int steps)
{
	expoCurve_t *result;

	if (steps < 2)
		return 0;

	result = malloc(sizeof(*result));

	result->steps = steps;
	result->curve = malloc(steps * sizeof(*result->curve));
	result->stepSize = 1.0 / (steps - 1);
	result->inputRange = inputRange;

	for (int i = 0; i < steps; i++) {
		result->curve[i] = pow(i * result->stepSize, power) * outputRange;
	}

	return result;
}

void expoCurveDestroy(expoCurve_t *curve)
{
	free(curve->curve);
	free(curve);
}
