#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "../../../src/blackbox_fielddefs.h"

#define FLIGHT_LOG_MAX_LOGS_IN_FILE 31
#define FLIGHT_LOG_MAX_FIELDS 128
#define FLIGHT_LOG_MAX_FRAME_LENGTH 256

#define FLIGHT_LOG_FIELD_INDEX_ITERATION 0
#define FLIGHT_LOG_FIELD_INDEX_TIME 1

typedef enum FirmwareType {
	FIRMWARE_TYPE_BASEFLIGHT = 0,
	FIRMWARE_TYPE_CLEANFLIGHT
} FirmwareType;

typedef struct FlightLogStatistics {
	uint32_t iFrameBytes, pFrameBytes, gFrameBytes, hFrameBytes, totalBytes;
	uint32_t numIFrames, numPFrames, numGFrames, numHFrames;

	// Number of frames that failed to decode:
	uint32_t numBrokenFrames;

	//Number of P frames that aren't usable because they were based on a frame that failed to decode:
	uint32_t numUnusablePFrames;

	//If our sampling rate is less than 1, we won't log every loop iteration, and that is accounted for here:
	uint32_t intentionallyAbsentFrames;

	int64_t fieldMaximum[FLIGHT_LOG_MAX_FIELDS];
	int64_t fieldMinimum[FLIGHT_LOG_MAX_FIELDS];

	uint32_t iFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
	uint32_t pFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
	uint32_t gFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
	uint32_t hFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
} flightLogStatistics_t;

struct flightLogPrivate_t;

typedef struct FlightLog {
	flightLogStatistics_t stats;

	int minthrottle, maxthrottle;
	unsigned int rcRate, yawRate;

	// Calibration constants from the hardware sensors:
	uint16_t acc_1G;
	float gyroScale;

	FirmwareType firmwareType;

	//Information about log sections:
	const char *logBegin[FLIGHT_LOG_MAX_LOGS_IN_FILE + 1];
	int logCount;

	unsigned int frameIntervalI;
	unsigned int frameIntervalPNum, frameIntervalPDenom;

	int mainFieldSigned[FLIGHT_LOG_MAX_FIELDS];
	int gpsFieldSigned[FLIGHT_LOG_MAX_FIELDS];

	int mainFieldCount;
	char *mainFieldNames[FLIGHT_LOG_MAX_FIELDS];

	int gpsFieldCount;
	char *gpsFieldNames[FLIGHT_LOG_MAX_FIELDS];

	struct flightLogPrivate_t *private;
} flightLog_t;

typedef void (*FlightLogMetadataReady)(flightLog_t *log);
typedef void (*FlightLogFrameReady)(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize);

flightLog_t* flightLogCreate(int fd);
bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, bool raw);
void flightLogDestroy(flightLog_t *log);

#endif
