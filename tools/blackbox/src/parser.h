#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define FLIGHT_LOG_MAX_LOGS_IN_FILE 31
#define FLIGHT_LOG_MAX_FIELDS 128
#define FLIGHT_LOG_MAX_FRAME_LENGTH 256

#define FLIGHT_LOG_FIELD_INDEX_ITERATION 0
#define FLIGHT_LOG_FIELD_INDEX_TIME 1

typedef enum FlightLogFieldPredictor
{
	//No prediction:
	FLIGHT_LOG_FIELD_PREDICTOR_0                = 0,

	//Predict that the field is the same as last frame:
	FLIGHT_LOG_FIELD_PREDICTOR_1                = 1,

	//Predict that the slope between this field and the previous item is the same as that between the past two history items:
	FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE    = 2,

	//Predict that this field is the same as the average of the last two history items:
	FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2        = 3,

	//Predict that this field is minthrottle
	FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE      = 4,

	//Predict that this field is the same as motor 0
	FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0          = 5,

	//This field always increments
	FLIGHT_LOG_FIELD_PREDICTOR_INC              = 6
} FlightLogFieldPredictor;

typedef enum FlightLogFieldEncoding
{
	FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       = 0,
	FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     = 1,
	FLIGHT_LOG_FIELD_ENCODING_U8              = 2,
	FLIGHT_LOG_FIELD_ENCODING_U16             = 3,
	FLIGHT_LOG_FIELD_ENCODING_U32             = 4,
	FLIGHT_LOG_FIELD_ENCODING_S8              = 5,
	FLIGHT_LOG_FIELD_ENCODING_S16             = 6,
	FLIGHT_LOG_FIELD_ENCODING_S32             = 7,
	FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       = 8
} FlightLogFieldEncoding;

typedef struct FlightLogStatistics {
	uint32_t iFrameBytes, pFrameBytes, totalBytes;
	uint32_t numIFrames, numPFrames;

	// Number of frames that failed to decode:
	uint32_t numBrokenFrames;

	/*
	 * Number of P frames that aren't usable because they were based on a frame that failed to decode:
	 */
	uint32_t numUnusablePFrames;

	int64_t fieldMaximum[FLIGHT_LOG_MAX_FIELDS];
	int64_t fieldMinimum[FLIGHT_LOG_MAX_FIELDS];

	uint32_t iFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
	uint32_t pFrameSizeCount[FLIGHT_LOG_MAX_FRAME_LENGTH];
} flightLogStatistics_t;

struct flightLogPrivate_t;

typedef struct FlightLog {
	flightLogStatistics_t stats;

	int minthrottle, maxthrottle;
	unsigned int rcRate, yawRate;

	// Calibration constants from the hardware sensors:
	uint16_t acc_1G;
	float gyroScale;

	//Information about log sections:
	const char *logBegin[FLIGHT_LOG_MAX_LOGS_IN_FILE + 1];
	int logCount;

	int fieldSigned[FLIGHT_LOG_MAX_FIELDS];
	int fieldCount;
	char *fieldNames[FLIGHT_LOG_MAX_FIELDS];

	struct flightLogPrivate_t *private;
} flightLog_t;

typedef void (*FlightLogMetadataReady)(flightLog_t *log);
typedef void (*FlightLogFrameReady)(flightLog_t *log, bool frameValid, int32_t *frame, int frameOffset, int frameSize);

flightLog_t* flightLogCreate(int fd);
bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, bool raw);
void flightLogDestroy(flightLog_t *log);

#endif
