/**
 * This tool reads in a flight log and re-encodes it using a private copy of the encoder. This allows experiments
 * to be run on improving the encoder's efficiency, and allows any changes to the encoder to be verified (by comparing
 * decoded logs against the ones produced original encoder).
 */

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
	#include "getopt.h"
#else
	#include <getopt.h>
#endif

#include "parser.h"

#define MAX_MOTORS 8
#define MAX_SERVOS 8

static const char blackboxHeader[] =
	"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
	"H Blackbox version:1\n"
	"H Data version:1\n";

// Some macros to make writing FLIGHT_LOG_FIELD_PREDICTOR_* constants shorter:
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)

#define PREDICT(x) STR(CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x))
#define ENCODING(x) STR(CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x))

/* These headers have info for all 8 motors on them, we'll trim the final fields off to match the number of motors in the mixer: */
static const char * const blackboxHeaderFields[] = {
	"H Field I name:"
		"loopIteration,time,"
		"axisP[0],axisP[1],axisP[2],"
		"axisI[0],axisI[1],axisI[2],"
		"axisD[0],axisD[1],axisD[2],"
		"rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],"
		"gyroData[0],gyroData[1],gyroData[2],"
		"accSmooth[0],accSmooth[1],accSmooth[2],"
		"motor[0],motor[1],motor[2],motor[3],"
		"motor[4],motor[5],motor[6],motor[7]",

	"H Field I signed:"
		/* loopIteration, time: */
		"0,0,"
		/* PIDs: */
		"1,1,1,1,1,1,1,1,1,"
		/* rcCommand[0..2] */
		"1,1,1,"
		/* rcCommand[3] (Throttle): */
		"0,"
		/* gyroData[0..2]: */
		"1,1,1,"
		/* accSmooth[0..2]: */
		"1,1,1,"
		/* Motor[0..7]: */
		"0,0,0,0,0,0,0,0",

	"H Field I predictor:"
		/* loopIteration, time: */
		PREDICT(0) "," PREDICT(0) ","
		/* PIDs: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* rcCommand[0..2] */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* rcCommand[3] (Throttle): */
		PREDICT(MINTHROTTLE) ","
		/* gyroData[0..2]: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* accSmooth[0..2]: */
		PREDICT(0) "," PREDICT(0) "," PREDICT(0) ","
		/* Motor[0]: */
		PREDICT(MINTHROTTLE) ","
		/* Motor[1..7]: */
		PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) ","
		PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) "," PREDICT(MOTOR_0) ","
		PREDICT(MOTOR_0),

     "H Field I encoding:"
		/* loopIteration, time: */
		ENCODING(UNSIGNED_VB) "," ENCODING(UNSIGNED_VB) ","
		/* PIDs: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[0..2] */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[3] (Throttle): */
		ENCODING(UNSIGNED_VB) ","
		/* gyroData[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* accSmooth[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* Motor[0]: */
		ENCODING(UNSIGNED_VB) ","
		/* Motor[1..7]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB),

	//Motors and gyros predict an average of the last two measurements (to reduce the impact of noise):
	"H Field P predictor:"
		/* loopIteration, time: */
		PREDICT(INC) "," PREDICT(STRAIGHT_LINE) ","
		/* PIDs: */
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		/* rcCommand[0..2] */
		PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) "," PREDICT(PREVIOUS) ","
		/* rcCommand[3] (Throttle): */
		PREDICT(PREVIOUS) ","
		/* gyroData[0..2]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		/* accSmooth[0..2]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		/* Motor[0]: */
		PREDICT(AVERAGE_2) ","
		/* Motor[1..7]: */
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) "," PREDICT(AVERAGE_2) ","
		PREDICT(AVERAGE_2),

	/* RC fields are encoded together as a group, everything else is signed since they're diffs: */
    "H Field P encoding:"
		/* loopIteration, time: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","
		/* PIDs: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(TAG2_3S32) "," ENCODING(TAG2_3S32) ","  ENCODING(TAG2_3S32) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* rcCommand[0..3] */
		ENCODING(TAG8_4S16) "," ENCODING(TAG8_4S16) ","  ENCODING(TAG8_4S16) ","
		ENCODING(TAG8_4S16) ","
		/* gyroData[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* accSmooth[0..2]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		/* Motor[0]: */
		ENCODING(SIGNED_VB) ","
		/* Motor[1..7]: */
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB) "," ENCODING(SIGNED_VB) ","  ENCODING(SIGNED_VB) ","
		ENCODING(SIGNED_VB)
};

/**
 * Additional fields to tack on to those above for tricopters (to record tail servo position)
 */
static const char * const blackboxAdditionalFieldsTricopter[] = {
	//Field I name
		"servo[5]",

	//Field I signed
		"0",

	//Field I predictor
		PREDICT(1500),

    //Field I encoding:
		ENCODING(SIGNED_VB),

	//Field P predictor:
		PREDICT(PREVIOUS),

    //Field P encoding:
		ENCODING(SIGNED_VB)
};

typedef struct blackbox_values_t {
	uint32_t time;

	int32_t axisP[3], axisI[3], axisD[3];

	int16_t rcCommand[4];
	int16_t gyroData[3];
	int16_t accSmooth[3];
	int16_t motor[MAX_MOTORS];
	int16_t servo[MAX_SERVOS];
} blackbox_values_t;

typedef struct mcfg_standin_t {
	uint16_t minthrottle, maxthrottle;
} mcfg_standin_t;

// Simulation of data that mw.c would normally provide:
mcfg_standin_t mcfg = {
	.minthrottle = 1150, .maxthrottle = 1850
};
int numberMotor = 0;

// Program options
int optionDebug;
char *optionFilename = 0;

uint32_t blackboxIteration, writtenBytes;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackbox_values_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackbox_values_t* blackboxHistory[3];

flightLogStatistics_t encodedStats;

static int isTricopter()
{
	return numberMotor == 3;
}

void blackboxWrite(uint8_t ch)
{
	putc(ch, stdout);

	writtenBytes++;
}

static void writeUnsignedVB(uint32_t value)
{
	//While this isn't the final byte (we can only write 7 bits at a time)
	while (value > 127) {
		blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
		value >>= 7;
	}
	blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
static void writeSignedVB(int32_t value)
{
	//ZigZag encode to make the value always positive
	writeUnsignedVB((uint32_t)((value << 1) ^ (value >> 31)));
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
static void writeTag2_3S32(int32_t *values) {
	static const int NUM_FIELDS = 3;

	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum { BITS_2  = 0};
	enum { BITS_4  = 1};
	enum { BITS_6  = 2};
	enum { BITS_32 = 3};

	enum { BYTES_1  = 0};
	enum { BYTES_2  = 1};
	enum { BYTES_3  = 2};
	enum { BYTES_4  = 3};

	int x;
	int selector = BITS_2, selector2;

	/*
	 * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
	 * below:
	 *
	 * Selector possibilities
	 *
	 * 2 bits per field  ss11 2233,
	 * 4 bits per field  ss00 1111 2222 3333
	 * 6 bits per field  ss11 1111 0022 2222 0033 3333
	 * 32 bits per field sstt tttt followed by fields of various byte counts
	 */
	for (x = 0; x < NUM_FIELDS; x++) {
		//Require more than 6 bits?
		if (values[x] >= 32 || values[x] < -32) {
			selector = BITS_32;
			break;
		}

		//Require more than 4 bits?
		if (values[x] >= 8 || values[x] < -8) {
			 if (selector < BITS_6)
				 selector = BITS_6;
		} else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
			if (selector < BITS_4)
				selector = BITS_4;
		}
	}

	switch (selector) {
		case BITS_2:
			blackboxWrite((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03));
		break;
		case BITS_4:
			blackboxWrite((selector << 6) | (values[0] & 0x0F));
			blackboxWrite((values[1] << 4) | (values[2] & 0x0F));
		break;
		case BITS_6:
			blackboxWrite((selector << 6) | (values[0] & 0x3F));
			blackboxWrite((uint8_t)values[1]);
			blackboxWrite((uint8_t)values[2]);
		break;
		case BITS_32:
			/*
			 * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
			 *
			 * Selector2 field possibilities
			 * 0 - 8 bits
			 * 1 - 16 bits
			 * 2 - 24 bits
			 * 3 - 32 bits
			 */
			selector2 = 0;

			//Encode in reverse order so the first field is in the low bits:
			for (x = NUM_FIELDS - 1; x >= 0; x--) {
				selector2 <<= 2;

				if (values[x] < 128 && values[x] >= -128)
					selector2 |= BYTES_1;
				else if (values[x] < 32768 && values[x] >= -32768)
					selector2 |= BYTES_2;
				else if (values[x] < 8388608 && values[x] >= -8388608)
					selector2 |= BYTES_3;
				else
					selector2 |= BYTES_4;
			}

			//Write the selectors
			blackboxWrite((selector << 6) | selector2);

			//And now the values according to the selectors we picked for them
			for (x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
				switch (selector2 & 0x03) {
					case BYTES_1:
						blackboxWrite(values[x]);
					break;
					case BYTES_2:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
					break;
					case BYTES_3:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
					break;
					case BYTES_4:
						blackboxWrite(values[x]);
						blackboxWrite(values[x] >> 8);
						blackboxWrite(values[x] >> 16);
						blackboxWrite(values[x] >> 24);
					break;
				}
			}
		break;
	}
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
static void writeTag8_4S16(int32_t *values) {

	//Need to be enums rather than const ints if we want to switch on them (due to being C)
	enum { FIELD_ZERO  = 0};
	enum { FIELD_4BIT  = 1};
	enum { FIELD_8BIT  = 2};
	enum { FIELD_16BIT = 3};

	uint8_t selector;
	int x;

	/*
	 * 4-bit fields can only be combined with their paired neighbor (there are two pairs), so choose a
	 * larger encoding if that's not possible.
	 */
	const uint8_t rcSelectorCleanup[16] = {
	//          Output selectors     <- Input selectors
		FIELD_ZERO << 2 | FIELD_ZERO,   // zero, zero
		FIELD_ZERO << 2 | FIELD_8BIT,   // zero, 4-bit
		FIELD_ZERO << 2 | FIELD_8BIT,   // zero, 8-bit
		FIELD_ZERO << 2 | FIELD_16BIT,  // zero, 16-bit
		FIELD_8BIT << 2 | FIELD_ZERO,   // 4-bit, zero
		FIELD_4BIT << 2 | FIELD_4BIT,   // 4-bit, 4-bit
		FIELD_8BIT << 2 | FIELD_8BIT,   // 4-bit, 8-bit
		FIELD_8BIT << 2 | FIELD_16BIT,  // 4-bit, 16-bit
		FIELD_8BIT << 2 | FIELD_ZERO,   // 8-bit, zero
		FIELD_8BIT << 2 | FIELD_8BIT,   // 8-bit, 4-bit
		FIELD_8BIT << 2 | FIELD_8BIT,   // 8-bit, 8-bit
		FIELD_8BIT << 2 | FIELD_16BIT,  // 8-bit, 16-bit
		FIELD_16BIT << 2 | FIELD_ZERO,  // 16-bit, zero
		FIELD_16BIT << 2 | FIELD_8BIT,  // 16-bit, 4-bit
		FIELD_16BIT << 2 | FIELD_8BIT,  // 16-bit, 8-bit
		FIELD_16BIT << 2 | FIELD_16BIT, // 16-bit, 16-bit
	};

	selector = 0;
	//Encode in reverse order so the first field is in the low bits:
	for (x = 3; x >= 0; x--) {
		selector <<= 2;

		if (values[x] == 0)
			selector |= FIELD_ZERO;
		else if (values[x] <= 7 && values[x] >= -8)
			selector |= FIELD_4BIT;
		else if (values[x] <= 127 && values[x] >= -128)
			selector |= FIELD_8BIT;
		else
			selector |= FIELD_16BIT;
	}

	selector = rcSelectorCleanup[selector & 0x0F] | (rcSelectorCleanup[selector >> 4] << 4);

	blackboxWrite(selector);

	for (x = 0; x < 4; x++, selector >>= 2) {
		switch (selector & 0x03) {
			case FIELD_4BIT:
				blackboxWrite((values[x] & 0x0F) | (values[x + 1] << 4));

				//We write two selector fields:
				x++;
				selector >>= 2;
			break;
			case FIELD_8BIT:
				blackboxWrite(values[x]);
			break;
			case FIELD_16BIT:
				blackboxWrite(values[x]);
				blackboxWrite(values[x] >> 8);
			break;
		}
	}
}

static void writeIntraframe(void)
{
	blackbox_values_t *blackboxCurrent = blackboxHistory[0];
	int x;

	blackboxWrite('I');

	writeUnsignedVB(blackboxIteration);
	writeUnsignedVB(blackboxCurrent->time);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisP[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisI[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisD[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->rcCommand[x]);

	writeUnsignedVB(blackboxCurrent->rcCommand[3] - mcfg.minthrottle); //Throttle lies in range [minthrottle..maxthrottle]

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->gyroData[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->accSmooth[x]);

	//Motors can be below minthrottle when disarmed, but that doesn't happen much
	writeUnsignedVB(blackboxCurrent->motor[0] - mcfg.minthrottle);

	//Motors tend to be similar to each other
	for (x = 1; x < numberMotor; x++)
		writeSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);

	if (isTricopter())
		writeSignedVB(blackboxHistory[0]->servo[5] - 1500);

	//Rotate our history buffers:

	//The current state becomes the new "before" state
	blackboxHistory[1] = blackboxHistory[0];
	//And since we have no other history, we also use it for the "before, before" state
	blackboxHistory[2] = blackboxHistory[0];
	//And advance the current state over to a blank space ready to be filled
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

static void writeInterframe(void)
{
	int x;
	int32_t deltas[4];

	blackbox_values_t *blackboxCurrent = blackboxHistory[0];
	blackbox_values_t *blackboxLast = blackboxHistory[1];

	blackboxWrite('P');

	//No need to store iteration count since its delta is always 1

	/*
	 * Since the difference between the difference between successive times will be nearly zero, use
	 * second-order differences.
	 */
	writeSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisP[x] - blackboxLast->axisP[x]);

	for (x = 0; x < 3; x++)
		deltas[x] = blackboxCurrent->axisI[x] - blackboxLast->axisI[x];

	/* 
	 * The PID I field changes very slowly, most of the time +-2, so use an encoding
	 * that can pack all three fields into one byte in that situation.
	 */
	writeTag2_3S32(deltas);
	
	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisD[x] - blackboxLast->axisD[x]);

	for (x = 0; x < 4; x++)
		deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];

	/*
	 * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
	 * can pack multiple values per byte:
	 */
	writeTag8_4S16(deltas);

	//Since gyros, accs and motors are noisy, base the prediction on the average of the history:
	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxHistory[0]->gyroData[x] - (blackboxHistory[1]->gyroData[x] + blackboxHistory[2]->gyroData[x]) / 2);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxHistory[0]->accSmooth[x] - (blackboxHistory[1]->accSmooth[x] + blackboxHistory[2]->accSmooth[x]) / 2);

	for (x = 0; x < numberMotor; x++)
		writeSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);

	if (isTricopter())
		writeSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}

/*
 * Treat each decoded frame as if it were a set of freshly read flight data ready to be
 * encoded.
 */
void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
	int x, src;
	uint32_t start = writtenBytes;
	unsigned int encodedFrameSize;
	blackbox_values_t *blackboxCurrent = blackboxHistory[0];

	(void) log;
	(void) frameOffset;
	(void) frameSize;
	(void) fieldCount;

	if (frameValid) {
		blackboxIteration = (uint32_t) frame[0];

		//Load the decoded frame into the buffer ready to be encoded
		src = 1;

		blackboxCurrent->time = (uint32_t) frame[src++];

		for (x = 0; x < 3; x++) {
			blackboxCurrent->axisP[x] = frame[src++];
		}

		for (x = 0; x < 3; x++) {
			blackboxCurrent->axisI[x] = frame[src++];
		}

		for (x = 0; x < 3; x++) {
			blackboxCurrent->axisD[x] = frame[src++];
		}

		for (x = 0; x < 4; x++) {
			blackboxCurrent->rcCommand[x] = frame[src++];
		}

		for (x = 0; x < 3; x++) {
			blackboxCurrent->gyroData[x] = frame[src++];
		}

		for (x = 0; x < 3; x++) {
			blackboxCurrent->accSmooth[x] = frame[src++];
		}

		for (x = 0; x < numberMotor; x++) {
			blackboxCurrent->motor[x] = frame[src++];
		}
		
		if (isTricopter())
			blackboxCurrent->servo[5] = frame[src++];

		if (frameType == 'I') {
			writeIntraframe();

			encodedFrameSize = writtenBytes - start;

			encodedStats.numIFrames++;
			encodedStats.iFrameBytes += encodedFrameSize;

			encodedStats.iFrameSizeCount[encodedFrameSize]++;
		} else if (frameType == 'P'){
			writeInterframe();

			encodedFrameSize = writtenBytes - start;

			encodedStats.numPFrames++;
			encodedStats.pFrameBytes += encodedFrameSize;

			encodedStats.pFrameSizeCount[encodedFrameSize]++;
		} else {
			fprintf(stderr, "Unknown frame type %c\n", (char) frameType);
			exit(-1);
		}
	}
}

void parseCommandlineOptions(int argc, char **argv)
{
	int c;

	while (1)
	{
		static struct option long_options[] = {
			{"debug", no_argument, &optionDebug, 1},
			{0, 0, 0, 0}
		};

		int option_index = 0;

		c = getopt_long (argc, argv, "", long_options, &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;
	}

	if (optind < argc)
		optionFilename = argv[optind];
}

// Print out a chart listing the numbers of frames in each size category
void printFrameSizeComparison(flightLogStatistics_t *oldStats, flightLogStatistics_t *newStats)
{
	// First determine the size bounds:
	int smallestSize = 0, largestSize = 255;

	for (int i = 0; i < 256; i++) {
		if (oldStats->iFrameSizeCount[i] || newStats->iFrameSizeCount[i]) {
			if (!smallestSize)
				smallestSize = i;
			largestSize = i;
		}
		if (oldStats->pFrameSizeCount[i] || newStats->pFrameSizeCount[i]) {
			if (!smallestSize)
				smallestSize = i;
			largestSize = i;
		}
	}

	fprintf(stderr, "\nFrame sizes\n");
	fprintf(stderr, "         Old       New       Old       New\n");
	fprintf(stderr, "Size   I count   I count   P count   P count\n");

	for (int i = smallestSize; i <= largestSize; i++) {
		fprintf(stderr, "%4d %9d %9d %9d %9d\n", i, oldStats->iFrameSizeCount[i], newStats->iFrameSizeCount[i],
				oldStats->pFrameSizeCount[i], newStats->pFrameSizeCount[i]);
	}
}

void printStats(flightLogStatistics_t *stats)
{
	uint32_t intervalMS = (uint32_t) ((stats->fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] - stats->fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]) / 1000);
	uint32_t totalBytes = stats->iFrameBytes + stats->pFrameBytes;
	uint32_t totalFrames = stats->numIFrames + stats->numPFrames;

	if (stats->numIFrames)
		fprintf(stderr, "I frames %7d %6.1f bytes avg %8d bytes total\n", stats->numIFrames, (double) stats->iFrameBytes / stats->numIFrames, stats->iFrameBytes);

	if (stats->numPFrames)
		fprintf(stderr, "P frames %7d %6.1f bytes avg %8d bytes total\n", stats->numPFrames, (double) stats->pFrameBytes / stats->numPFrames, stats->pFrameBytes);

	if (totalFrames)
		fprintf(stderr, "Frames %9d %6.1f bytes avg %8d bytes total\n", totalFrames, (double) totalBytes / totalFrames, totalBytes);
	else
		fprintf(stderr, "Frames %8d\n", 0);

	if (stats->numBrokenFrames)
		fprintf(stderr, "%d frames failed to decode (%.2f%%)\n", stats->numBrokenFrames, (double) stats->numBrokenFrames / (stats->numBrokenFrames + stats->numIFrames + stats->numPFrames) * 100);

	fprintf(stderr, "IntervalMS %u Total bytes %u\n", intervalMS, stats->totalBytes);

	if (intervalMS > 0) {
		fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
				(unsigned int) (((int64_t) totalFrames * 1000) / intervalMS),
				(unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
				(unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
	}
}

void writeHeader()
{
	int motorsToRemove = 8 - numberMotor;
	const char *additionalHeader;

	printf("%s", blackboxHeader);
	writtenBytes = strlen(blackboxHeader);

	for (unsigned int i = 0; i < sizeof(blackboxHeaderFields) / sizeof(blackboxHeaderFields[0]); i++) {
		int endIndex = strlen(blackboxHeaderFields[i]) - (i == 0 ? strlen(",motor[x]") : strlen(",x")) * motorsToRemove;

		for (int j = 0; j < endIndex; j++)
			blackboxWrite(blackboxHeaderFields[i][j]);

		if (isTricopter()) {
			//Add fields to the end for the tail servo
			blackboxWrite(',');

			for (additionalHeader = blackboxAdditionalFieldsTricopter[i]; *additionalHeader; additionalHeader++)
				blackboxWrite(*additionalHeader);
		}

		blackboxWrite('\n');
	}
}

void onMetadataReady(flightLog_t *log)
{
	int i;

	numberMotor = 0;

	for (i = 0; i < log->mainFieldCount; i++) {
		if (strncmp(log->mainFieldNames[i], "motor[", strlen("motor[")) == 0) {
			int motorIndex = atoi(log->mainFieldNames[i] + strlen("motor["));

			if (motorIndex + 1 > numberMotor)
				numberMotor = motorIndex + 1;
		}
	}

	writeHeader();
}

int main(int argc, char **argv)
{
	FILE *input;
	flightLog_t *log;

	parseCommandlineOptions(argc, argv);

	if (!optionFilename) {
		fprintf(stderr, "Missing log filename argument\n");
		return -1;
	}

	input = fopen(optionFilename, "rb");

	if (!input) {
		fprintf(stderr, "Failed to open input file!\n");
		return -1;
	}

	blackboxHistory[0] = &blackboxHistoryRing[0];
	blackboxHistory[1] = &blackboxHistoryRing[1];
	blackboxHistory[2] = &blackboxHistoryRing[2];

	log = flightLogCreate(fileno(input));

	flightLogParse(log, 0, onMetadataReady, onFrameReady, 0);

	encodedStats.totalBytes = writtenBytes;
	encodedStats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME] = log->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME];
	encodedStats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] = log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME];

	fprintf(stderr, "Logged time %u seconds\n", (uint32_t)((log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] - log->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]) / 1000000));

	fprintf(stderr, "\nOriginal statistics\n");
	printStats(&log->stats);

	fprintf(stderr, "\nNew statistics\n");
	printStats(&encodedStats);

	printFrameSizeComparison(&log->stats, &encodedStats);

	flightLogDestroy(log);

	return 0;
}
