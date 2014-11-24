#ifndef DATAPOINTS_H_
#define DATAPOINTS_H_

#include <stdint.h>

typedef struct Datapoints {
	int fieldCount, frameCount;
	char **fieldNames;

	uint8_t *framePresent;
	int32_t *frames;
	int64_t *frameTime;
} Datapoints;

Datapoints *datapointsCreate(int fieldCount, char **fieldNames, int frameCount);

bool datapointsGetFrameAtIndex(Datapoints *points, int frameIndex, int64_t *frameTime, int32_t *frame);

bool datapointsGetFieldAtIndex(Datapoints *points, int frameIndex, int fieldIndex, int32_t *frameValue);
bool datapointsSetFieldAtIndex(Datapoints *points, int frameIndex, int fieldIndex, int32_t frameValue);

bool datapointsGetTimeAtIndex(Datapoints *points, int frameIndex, int64_t *frameTime);

int datapointsFindFrameAtTime(Datapoints *points, int64_t time);
bool datapointsSetFrame(Datapoints *points, int frameIndex, int64_t frameTime, int32_t *frame);

void datapointsSmoothField(Datapoints *points, int fieldIndex, int windowSize);

#endif
