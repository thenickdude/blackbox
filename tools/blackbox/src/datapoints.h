#ifndef DATAPOINTS_H_
#define DATAPOINTS_H_

#include <stdint.h>

typedef struct datapoints_t {
	int fieldCount, frameCount;
	char **fieldNames;

	uint8_t *framePresent;
	int32_t *frames;
	int64_t *frameTime;
} datapoints_t;

datapoints_t *datapointsCreate(int fieldCount, char **fieldNames, int frameCount);

bool datapointsGetFrameAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime, int32_t *frame);

bool datapointsGetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int32_t *frameValue);
bool datapointsSetFieldAtIndex(datapoints_t *points, int frameIndex, int fieldIndex, int32_t frameValue);

bool datapointsGetTimeAtIndex(datapoints_t *points, int frameIndex, int64_t *frameTime);

int datapointsFindFrameAtTime(datapoints_t *points, int64_t time);
bool datapointsSetFrame(datapoints_t *points, int frameIndex, int64_t frameTime, int32_t *frame);

void datapointsSmoothField(datapoints_t *points, int fieldIndex, int windowSize);

#endif
