#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "datapoints.h"
#include "parser.h"

Datapoints *datapointsCreate(int fieldCount, char **fieldNames, int frameCount) {
	Datapoints *result = (Datapoints*) malloc(sizeof(Datapoints));

	result->fieldCount = fieldCount;
	result->fieldNames = fieldNames;
	result->frameCount = frameCount;

	result->frames = malloc(sizeof(*result->frames) * fieldCount * frameCount);
	result->framePresent = calloc(1, sizeof(*result->framePresent) * frameCount);
	result->frameTime = calloc(1, sizeof(*result->frameTime) * frameCount);

	return result;
}

void datapointsSmoothField(Datapoints *points, int fieldIndex, int windowSize)
{
	// How many frames are inside the history window (this is less than windowSize at the beginning and end of the buffer)
	int historySize = 0;

	// How many of the frames in the history actually have a valid value in them (so we can average only those)
	int valuesInHistory = 0;

	int64_t accumulator = 0;

	if (windowSize < 2)
		return;

	int32_t *history = (int32_t*) malloc(sizeof(*history) * windowSize);
	int historyHead = 0; //Points to the next location to insert into
	int historyTail = 0; //Points to the last value in the history windowSize

	for (int frameIndex = 0; frameIndex < points->frameCount; frameIndex++) {
		//If history is full, oldest value falls out of the window
		if (historySize >= windowSize) {
			if (points->framePresent[historyTail]) {
				accumulator -= history[historyTail];
				valuesInHistory--;
			}

			historyTail = (historyTail + 1) % windowSize;
			historySize--;
		}

		if (points->framePresent[frameIndex]) {
			int32_t fieldValue = (int32_t) points->frames[points->fieldCount * frameIndex + fieldIndex];
			accumulator += fieldValue;

			history[historyHead] = fieldValue;
			valuesInHistory++;
		}

		historyHead = (historyHead + 1) % windowSize;
		historySize++;

		// Store the average of the history window into the frame in the center of the window
		int centerIndex = frameIndex - historySize / 2;

		if (points->framePresent[centerIndex]) {
			points->frames[points->fieldCount * centerIndex + fieldIndex] = accumulator / valuesInHistory;
		}
	}

	free(history);
}

/**
 * Find the index of the latest frame whose time is equal to or later than 'time'.
 *
 * Returns -1 if the time is before any frame in the datapoints.
 */
int datapointsFindFrameAtTime(Datapoints *points, int64_t time)
{
	int i, lastGoodFrame = -1;

	//TODO make me a binary search
	for (i = 0; i < points->frameCount; i++) {
		if (points->framePresent[i]) {
			if (time < points->frameTime[i]) {
				return lastGoodFrame;
			}
			lastGoodFrame = i;
		}
	}

	return lastGoodFrame;
}

bool datapointsGetFrameAtIndex(Datapoints *points, int frameIndex, int64_t *frameTime, int32_t *frame)
{
	if (frameIndex < 0 || frameIndex >= points->frameCount || !points->framePresent[frameIndex])
		return false;

	memcpy(frame, points->frames + frameIndex * points->fieldCount, points->fieldCount * sizeof(*points->frames));
	*frameTime = points->frameTime[frameIndex];

	return true;
}

bool datapointsGetFieldAtIndex(Datapoints *points, int frameIndex, int fieldIndex, int32_t *frameValue)
{
	if (frameIndex < 0 || frameIndex >= points->frameCount || !points->framePresent[frameIndex])
		return false;

	*frameValue = points->frames[frameIndex * points->fieldCount + fieldIndex];

	return true;
}

bool datapointsGetTimeAtIndex(Datapoints *points, int frameIndex, int64_t *frameTime)
{
	if (frameIndex < 0 || frameIndex >= points->frameCount || !points->framePresent[frameIndex])
		return false;

	*frameTime = points->frameTime[frameIndex];

	return true;
}

/**
 * Set the data for the frame with the given index. The second field of the frame is expected to be a timestamp
 * (if you want to be able to find frames at given times).
 */
bool datapointsSetFrame(Datapoints *points, int frameIndex, int64_t frameTime, int32_t *frame)
{
	if (frameIndex < 0 || frameIndex >= points->frameCount)
		return false;

	memcpy(points->frames + frameIndex * points->fieldCount, frame, points->fieldCount * sizeof(*points->frames));
	points->frameTime[frameIndex] = frameTime;
	points->framePresent[frameIndex] = true;

	return true;
}
