#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <sys/stat.h>

// Support for memory-mapped files:
#ifdef WIN32
	#include <windows.h>
	#include <io.h>
#else
	// Posix-y systems:
	#include <sys/mman.h>
#endif

#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "parser.h"

#define LOG_START_MARKER "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"

typedef enum ParserState
{
	PARSER_STATE_HEADER = 0,
	PARSER_STATE_BEFORE_FIRST_FRAME,
	PARSER_STATE_DATA
} ParserState;

typedef struct flightLogPrivate_t
{
	char *fieldNamesCombined;

	//Information about fields which we need to decode them properly
	int fieldPPredictor[FLIGHT_LOG_MAX_FIELDS];
	int fieldPEncoding[FLIGHT_LOG_MAX_FIELDS];
	int fieldIPredictor[FLIGHT_LOG_MAX_FIELDS];
	int fieldIEncoding[FLIGHT_LOG_MAX_FIELDS];

	int dataVersion;
	int motor0Index;

	int32_t blackboxHistoryRing[2][FLIGHT_LOG_MAX_FIELDS];
	int32_t* mainHistory[2];

	int fd;

	//The start of the entire log file:
	const char *logData;

	//The section of the file which is currently being examined:
	const char *logStart, *logEnd, *logPos;

	//Set to true if we attempt to read from the log when it is already exhausted
	bool eof;
} flightLogPrivate_t;

static int readChar(flightLog_t *log)
{
	if (log->private->logPos < log->private->logEnd) {
		int result = (uint8_t) *log->private->logPos;
		log->private->logPos++;
		return result;
	}

	log->private->eof = true;
	return EOF;
}

static void unreadChar(flightLog_t *log, int c)
{
	(void) c;

	log->private->logPos--;
}

static void parseFieldNames(flightLog_t *log, const char *names)
{
	char *start, *end;
	bool done = false;

	log->private->fieldNamesCombined = strdup(names);

	start = log->private->fieldNamesCombined;

	while (!done && *start) {
		end = start;

		do {
			end++;
		} while (*end != ',' && *end != 0);

		log->mainFieldNames[log->mainFieldCount++] = start;

		if (*end == 0)
			done = true;

		*end = 0;

		start = end + 1;
	}
}

static void parseCommaSeparatedIntegers(char *line, int *target, int maxCount)
{
	char *start, *end;
	bool done = false;

	start = line;

	while (!done && *start && maxCount > 0) {
		end = start + 1;

		while (*end != ',' && *end != 0)
			end++;

		if (*end == 0)
			done = true;

		*end = 0;

		*target = atoi(start);
		target++;
		maxCount--;

		start = end + 1;
	}
}

static void parseHeader(flightLog_t *log)
{
	char *fieldName, *fieldValue;
	const char *lineStart, *lineEnd, *separatorPos;
	int i, c;
	char valueBuffer[1024];
	union {
		float f;
		uint32_t u;
	} floatConvert;

	if (*log->private->logPos != ' ')
		return;

	//Skip the space
	log->private->logPos++;

	lineStart = log->private->logPos;
	separatorPos = 0;

	for (i = 0; i < 1024; i++) {
		c = readChar(log);

		if (c == ':' && !separatorPos)
			separatorPos = log->private->logPos - 1;

		if (c == '\n')
			break;

		if (c == EOF || c == '\0')
			// Line ended before we saw a newline or it has binary stuff in there that shouldn't be there
			return;
	}

	if (!separatorPos)
		return;

	lineEnd = log->private->logPos;

	//Make a duplicate copy of the line so we can null-terminate the two parts
	memcpy(valueBuffer, lineStart, lineEnd - lineStart);

	fieldName = valueBuffer;
	valueBuffer[separatorPos - lineStart] = '\0';

	fieldValue = valueBuffer + (separatorPos - lineStart) + 1;
	valueBuffer[lineEnd - lineStart - 1] = '\0';

	if (strcmp(fieldName, "Field I name") == 0) {
		parseFieldNames(log, fieldValue);

		for (i = 0; i < log->mainFieldCount; i++) {
			if (strcmp(log->mainFieldNames[i], "motor[0]") == 0) {
				log->private->motor0Index = i;
				break;
			}
		}
	} else if (strcmp(fieldName, "Field P predictor") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldPPredictor, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field P encoding") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldPEncoding, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field I predictor") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldIPredictor, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field I encoding") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldIEncoding, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field I signed") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->mainFieldSigned, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "I interval") == 0) {
		log->frameIntervalI = atoi(fieldValue);
		if (log->frameIntervalI < 1)
			log->frameIntervalI = 1;
	} else if (strcmp(fieldName, "P interval") == 0) {
		char *slashPos = strchr(fieldValue, '/');

		if (slashPos) {
			log->frameIntervalPNum = atoi(fieldValue);
			log->frameIntervalPDenom = atoi(slashPos + 1);
		}
	} else if (strcmp(fieldName, "Data version") == 0) {
		log->private->dataVersion = atoi(fieldValue);
	} else if (strcmp(fieldName, "Firmware type") == 0) {
		if (strcmp(fieldValue, "Cleanflight") == 0)
			log->firmwareType = FIRMWARE_TYPE_CLEANFLIGHT;
		else
			log->firmwareType = FIRMWARE_TYPE_BASEFLIGHT;
	} else if (strcmp(fieldName, "minthrottle") == 0) {
		log->minthrottle = atoi(fieldValue);
	} else if (strcmp(fieldName, "maxthrottle") == 0) {
		log->maxthrottle = atoi(fieldValue);
	} else if (strcmp(fieldName, "rcRate") == 0) {
		log->rcRate = atoi(fieldValue);
    } else if (strcmp(fieldName, "vbatscale") == 0) {
        log->vbatscale = atoi(fieldValue);
    } else if (strcmp(fieldName, "vbatref") == 0) {
        log->vbatref = atoi(fieldValue);
    } else if (strncmp(fieldName, "vbatcellvoltage", strlen("vbatcellvoltage")) == 0) {
        int vbatcellvoltage[3];
        parseCommaSeparatedIntegers(fieldValue, vbatcellvoltage, 3);

        log->vbatmincellvoltage = vbatcellvoltage[0];
        log->vbatwarningcellvoltage = vbatcellvoltage[1];
        log->vbatmaxcellvoltage = vbatcellvoltage[2];
	} else if (strcmp(fieldName, "gyro.scale") == 0) {
		floatConvert.u = strtoul(fieldValue, 0, 16);

		log->gyroScale = floatConvert.f;

		/* Baseflight uses a gyroScale that'll give radians per microsecond as output, whereas Cleanflight produces degrees
		 * per second and leaves the conversion to radians per us to the IMU. Let's just convert Cleanflight's scale to
		 * match Baseflight so we can use Baseflight's IMU for both: */

		if (log->firmwareType == FIRMWARE_TYPE_CLEANFLIGHT) {
			log->gyroScale = (float) (log->gyroScale * (M_PI / 180.0) * 0.000001);
		}
	} else if (strcmp(fieldName, "acc_1G") == 0) {
		log->acc_1G = atoi(fieldValue);
	}
}

static uint32_t readUnsignedVB(flightLog_t *log)
{
	int i, c, shift = 0;
	uint32_t result = 0;

	// 5 bytes is enough to encode 32-bit unsigned quantities
	for (i = 0; i < 5; i++) {
		c = readChar(log);

		if (c == EOF) {
			return 0;
		}

		result = result | ((c & ~0x80) << shift);

		//Final byte?
		if (c < 128) {
			return result;
		}

		shift += 7;
	}

	// This VB-encoded int is too long!
	return 0;
}

static int32_t readSignedVB(flightLog_t *log)
{
	uint32_t i = readUnsignedVB(log);

	// Apply ZigZag decoding to recover the signed value
	return (i >> 1) ^ -(int32_t) (i & 1);
}

static int32_t signExtend24Bit(uint32_t u)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (u & 0x800000) ? (int32_t) (u | 0xFF000000) : (int32_t) u;
}

static int32_t signExtend12Bit(uint16_t word)
{
    //If sign bit is set, fill the top bits with 1s to sign-extend
    return (word & 0x800) ? (int32_t) (int16_t) (word | 0xF000) : word;
}

static int32_t signExtend6Bit(uint8_t byte)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (byte & 0x20) ? (int32_t) (int8_t) (byte | 0xC0) : byte;
}

static int32_t signExtend4Bit(uint8_t nibble)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (nibble & 0x08) ? (int32_t) (int8_t) (nibble | 0xF0) : nibble;
}

static int32_t signExtend2Bit(uint8_t byte)
{
	//If sign bit is set, fill the top bits with 1s to sign-extend
	return (byte & 0x02) ? (int32_t) (int8_t) (byte | 0xFC) : byte;
}

static void readTag2_3S32(flightLog_t *log, int32_t *values)
{
	uint8_t leadByte;
	uint8_t byte1, byte2, byte3, byte4;
	int i;

	leadByte = readChar(log);

	// Check the selector in the top two bits to determine the field layout
	switch (leadByte >> 6) {
		case 0:
			// 2-bit fields
			values[0] = signExtend2Bit((leadByte >> 4) & 0x03);
			values[1] = signExtend2Bit((leadByte >> 2) & 0x03);
			values[2] = signExtend2Bit(leadByte & 0x03);
		break;
		case 1:
			// 4-bit fields
			values[0] = signExtend4Bit(leadByte & 0x0F);

			leadByte = readChar(log);

			values[1] = signExtend4Bit(leadByte >> 4);
			values[2] = signExtend4Bit(leadByte & 0x0F);
		break;
		case 2:
			// 6-bit fields
			values[0] = signExtend6Bit(leadByte & 0x3F);

			leadByte = readChar(log);
			values[1] = signExtend6Bit(leadByte & 0x3F);

			leadByte = readChar(log);
			values[2] = signExtend6Bit(leadByte & 0x3F);
		break;
		case 3:
			// Fields are 8, 16 or 24 bits, read selector to figure out which field is which size

			for (i = 0; i < 3; i++) {
				switch (leadByte & 0x03) {
					case 0: // 8-bit
						byte1 = readChar(log);

						// Sign extend to 32 bits
						values[i] = (int32_t) (int8_t) (byte1);
					break;
					case 1: // 16-bit
						byte1 = readChar(log);
						byte2 = readChar(log);

						// Sign extend to 32 bits
						values[i] = (int32_t) (int16_t) (byte1 | (byte2 << 8));
					break;
					case 2: // 24-bit
						byte1 = readChar(log);
						byte2 = readChar(log);
						byte3 = readChar(log);

						// Cause sign-extension by using arithmetic right-shift
						values[i] = signExtend24Bit(byte1 | (byte2 << 8) | (byte3 << 16));
					break;
					case 3: // 32-bit
						byte1 = readChar(log);
						byte2 = readChar(log);
						byte3 = readChar(log);
						byte4 = readChar(log);

						values[i] = (int32_t) (byte1 | (byte2 << 8) | (byte3 << 16) | (byte4 << 24));
					break;
				}

				leadByte >>= 2;
			}
		break;
	}
}

static void readTag8_4S16_v1(flightLog_t *log, int32_t *values)
{
	uint8_t selector, combinedChar;
	uint8_t char1, char2;
	int i;

	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	selector = readChar(log);

	//Read the 4 values from the stream
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				values[i] = 0;
			break;
			case FIELD_4BIT: // Two 4-bit fields
				combinedChar = (uint8_t) readChar(log);

				values[i] = signExtend4Bit(combinedChar & 0x0F);

				i++;
				selector >>= 2;

				values[i] = signExtend4Bit(combinedChar >> 4);
			break;
			case FIELD_8BIT: // 8-bit field
				//Sign extend...
				values[i] = (int32_t) (int8_t) readChar(log);
			break;
			case FIELD_16BIT: // 16-bit field
				char1 = readChar(log);
				char2 = readChar(log);

				//Sign extend...
				values[i] = (int16_t) (char1 | (char2 << 8));
			break;
		}

		selector >>= 2;
	}
}

static void readTag8_4S16_v2(flightLog_t *log, int32_t *values)
{
	uint8_t selector;
	uint8_t char1, char2;
	uint8_t buffer;
	int nibbleIndex;

	int i;

	enum {
		FIELD_ZERO  = 0,
		FIELD_4BIT  = 1,
		FIELD_8BIT  = 2,
		FIELD_16BIT = 3
	};

	selector = readChar(log);

	//Read the 4 values from the stream
	nibbleIndex = 0;
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case FIELD_ZERO:
				values[i] = 0;
			break;
			case FIELD_4BIT:
				if (nibbleIndex == 0) {
					buffer = (uint8_t) readChar(log);
					values[i] = signExtend4Bit(buffer >> 4);
					nibbleIndex = 1;
				} else {
					values[i] = signExtend4Bit(buffer & 0x0F);
					nibbleIndex = 0;
				}
			break;
			case FIELD_8BIT:
				if (nibbleIndex == 0) {
					//Sign extend...
					values[i] = (int32_t) (int8_t) readChar(log);
				} else {
					char1 = buffer << 4;
					buffer = (uint8_t) readChar(log);

					char1 |= buffer >> 4;
					values[i] = (int32_t) (int8_t) char1;
				}
			break;
			case FIELD_16BIT:
				if (nibbleIndex == 0) {
					char1 = (uint8_t) readChar(log);
					char2 = (uint8_t) readChar(log);

					//Sign extend...
					values[i] = (int16_t) (uint16_t) ((char1 << 8) | char2);
				} else {
					/*
					 * We're in the low 4 bits of the current buffer, then one byte, then the high 4 bits of the next
					 * buffer.
					 */
					char1 = (uint8_t) readChar(log);
					char2 = (uint8_t) readChar(log);

					values[i] = (int16_t) (uint16_t) ((buffer << 12) | (char1 << 4) | (char2 >> 4));

					buffer = char2;
				}
			break;
		}

		selector >>= 2;
	}
}

static void readTag8_8SVB(flightLog_t *log, int32_t *values, int valueCount)
{
    uint8_t header;

    if (valueCount == 1) {
        values[0] = readSignedVB(log);
    } else {
        header = (uint8_t) readChar(log);

        for (int i = 0; i < 8; i++, header >>= 1)
            values[i] = (header & 0x01) ? readSignedVB(log) : 0;
    }
}

static void parseIntraframe(flightLog_t *log, bool raw)
{
	int i;

	log->private->mainHistory[0] = &log->private->blackboxHistoryRing[0][0];
	log->private->mainHistory[1] = &log->private->blackboxHistoryRing[0][0];

	for (i = 0; i < log->mainFieldCount; i++) {
		uint32_t value;

		switch (log->private->fieldIEncoding[i]) {
			case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
				value = (uint32_t) readSignedVB(log);
			break;
			case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
				value = readUnsignedVB(log);
			break;
			case FLIGHT_LOG_FIELD_ENCODING_NEG_12BIT:
			    value = (uint32_t) -signExtend12Bit(readUnsignedVB(log));
			break;
			default:
				fprintf(stderr, "Unsupported I-field encoding %d\n", log->private->fieldIEncoding[i]);
				exit(-1);
		}

		if (!raw) {
			//Not many predictors can be used in I-frames since they can't depend on any other frame
			switch (log->private->fieldIPredictor[i]) {
				case FLIGHT_LOG_FIELD_PREDICTOR_0:
					//No-op
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE:
					value += log->minthrottle;
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_1500:
					value += 1500;
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0:
					if (log->private->motor0Index < 0) {
						fprintf(stderr, "Attempted to base I-field prediction on motor0 before it was read\n");
						exit(-1);
					}
					value += (uint32_t) log->private->mainHistory[0][log->private->motor0Index];
				break;
				case FLIGHT_LOG_FIELD_PREDICTOR_VBATREF:
				    value += log->vbatref;
				break;
				default:
					fprintf(stderr, "Unsupported I-field predictor %d\n", log->private->fieldIPredictor[i]);
					exit(-1);
			}
		}

		log->private->mainHistory[0][i] = (int32_t) value;
	}
}

/**
 * Take the raw value for an inter-field, apply the prediction that is configured for it, and return it.
 *
 */
static int32_t applyInterPrediction(flightLog_t *log, int fieldIndex, int predictor, uint32_t value)
{
	switch (predictor) {
		case FLIGHT_LOG_FIELD_PREDICTOR_0:
			// No correction to apply
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS:
			value += (uint32_t) log->private->mainHistory[0][fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE:
			value += 2 * (uint32_t) log->private->mainHistory[0][fieldIndex] - (uint32_t) log->private->mainHistory[1][fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2:
			if (log->mainFieldSigned[fieldIndex])
				value += (uint32_t) ((int32_t) ((uint32_t) log->private->mainHistory[0][fieldIndex] + (uint32_t) log->private->mainHistory[1][fieldIndex]) / 2);
			else
				value += ((uint32_t) log->private->mainHistory[0][fieldIndex] + (uint32_t) log->private->mainHistory[1][fieldIndex]) / 2;
		break;
		default:
			fprintf(stderr, "Unsupported P-field predictor %d\n", log->private->fieldPPredictor[fieldIndex]);
			exit(-1);
	}

	return (int32_t) value;
}

/**
 * Should a frame with the given index exist in this log (based on the user's selection of sampling rates)?
 */
static int shouldHaveFrame(flightLog_t *log, int32_t frameIndex)
{
	return (frameIndex % log->frameIntervalI + log->frameIntervalPNum - 1) % log->frameIntervalPDenom < log->frameIntervalPNum;
}

static void parseInterframe(flightLog_t *log, bool raw)
{
	int i, j;
	int groupCount;

	// The new frame gets stored over the top of the current oldest history
	int32_t *newFrame = log->private->mainHistory[0] == &log->private->blackboxHistoryRing[0][0] ? &log->private->blackboxHistoryRing[1][0] : &log->private->blackboxHistoryRing[0][0];
	uint32_t frameIndex;
	uint32_t skippedFrames = 0;

	//Work out how many frames we skipped to get to this one, based on the log sampling rate
	for (frameIndex = log->private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++) {
		skippedFrames++;
	}
	log->stats.intentionallyAbsentFrames += skippedFrames;

	for (i = 0; i < log->mainFieldCount; i++) {
		uint32_t value;
		uint32_t values[8];

		if (log->private->fieldPPredictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_INC) {
			newFrame[i] = log->private->mainHistory[0][i] + 1 + skippedFrames;
		} else {
			switch (log->private->fieldPEncoding[i]) {
				case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
					value = (uint32_t) readSignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
					value = readUnsignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16:
					if (log->private->dataVersion < 2)
						readTag8_4S16_v1(log, (int32_t*)values);
					else
						readTag8_4S16_v2(log, (int32_t*)values);

					//Apply the predictors for the fields:
					for (j = 0; j < 3; j++) {
						// ^ But don't process the final value, allow the regular handler after the 'case' to do that
						newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], values[j]);
						i++;
					}
					value = values[3];
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32:
					readTag2_3S32(log, (int32_t*)values);

					//Apply the predictors for the fields:
					for (j = 0; j < 2; j++) {
						// ^ But don't process the final value, allow the regular handler after the 'case' to do that
						newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], values[j]);
						i++;
					}
					value = values[2];
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB:
				    //How many fields are in this encoded group? Check the subsequent field encodings:
				    for (j = i + 1; j < i + 8 && j < log->mainFieldCount; j++)
				        if (log->private->fieldPEncoding[j] != FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB)
				            break;

				    groupCount = j - i;

				    readTag8_8SVB(log, (int32_t*) values, groupCount);

                    // Don't process the final value, allow the regular handler after the 'case' to do that
				    for (j = 0; j < groupCount - 1; j++) {
                        newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], values[j]);
                        i++;
				    }
				    value = values[groupCount - 1];
				break;
				case FLIGHT_LOG_FIELD_ENCODING_NULL:
					continue;
				break;
				default:
					fprintf(stderr, "Unsupported P-field encoding %d\n", log->private->fieldPEncoding[i]);
					exit(-1);
			}

			newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], value);
		}
	}

	log->private->mainHistory[1] = log->private->mainHistory[0];
	log->private->mainHistory[0] = newFrame;
}

/**
 * TODO
 */
static void parseGPSFrame(flightLog_t *log, bool raw)
{
	(void) raw;

	readUnsignedVB(log);
	readSignedVB(log);
	readSignedVB(log);
	readUnsignedVB(log);
	readUnsignedVB(log);
}

/**
 * TODO
 */
static void parseGPSHomeFrame(flightLog_t *log, bool raw)
{
	(void) raw;

	readSignedVB(log);
	readSignedVB(log);
}

static void updateFieldStatistics(flightLog_t *log, int32_t *fields)
{
	int i;

	if (log->stats.numIFrames + log->stats.numPFrames <= 1) {
		//If this is the first frame, there are no minimums or maximums in the stats to compare with
		for (i = 0; i < log->mainFieldCount; i++) {
			if (log->mainFieldSigned[i]) {
				log->stats.fieldMaximum[i] = fields[i];
				log->stats.fieldMinimum[i] = fields[i];
			} else {
				log->stats.fieldMaximum[i] = (uint32_t) fields[i];
				log->stats.fieldMinimum[i] = (uint32_t) fields[i];
			}
		}
	} else {
		for (i = 0; i < log->mainFieldCount; i++) {
			if (log->mainFieldSigned[i]) {
				log->stats.fieldMaximum[i] = fields[i] > log->stats.fieldMaximum[i] ? fields[i] : log->stats.fieldMaximum[i];
				log->stats.fieldMinimum[i] = fields[i] < log->stats.fieldMinimum[i] ? fields[i] : log->stats.fieldMinimum[i];
			} else {
				log->stats.fieldMaximum[i] = (uint32_t) fields[i] > log->stats.fieldMaximum[i] ? (uint32_t) fields[i] : log->stats.fieldMaximum[i];
				log->stats.fieldMinimum[i] = (uint32_t) fields[i] < log->stats.fieldMinimum[i] ? (uint32_t) fields[i] : log->stats.fieldMinimum[i];
			}
		}
	}
}

static void updateFrameSizeStats(uint32_t *counts, unsigned int lastFrameSize)
{
	//Don't write out of bounds
	if (lastFrameSize < FLIGHT_LOG_MAX_FRAME_LENGTH) {
		counts[lastFrameSize]++;
	}
}

/**
 * Just like strstr, but for binary strings. Not available on all platforms, so reimplemented here.
 */
void* memmem(const void *haystack, size_t haystackLen, const void *needle, size_t needleLen)
{
	if (needleLen <= haystackLen) {
		const char* c_haystack = (char*)haystack;
		const char* c_needle = (char*)needle;

		for (const char *pos = c_haystack; pos <= c_haystack + haystackLen - needleLen; pos++) {
			if (*pos == *c_needle && memcmp(pos, c_needle, needleLen) == 0)
				return (void*)pos;
		}
	}

	return NULL;
}

unsigned int flightLogVbatToMillivolts(flightLog_t *log, uint16_t vbat)
{
    // ADC is 12 bit (i.e. max 0xFFF), voltage reference is 3.3V, vbatscale is premultiplied by 100
    return (vbat * 33 * log->vbatscale) / 0xFFF;
}

int flightLogEstimateNumCells(flightLog_t *log)
{
    int i;
    int refVoltage;

    refVoltage = flightLogVbatToMillivolts(log, log->vbatref) / 100;
    for (i = 1; i < 8; i++) {
        if (refVoltage < i * log->vbatmaxcellvoltage)
            break;
    }

    return i;
}

flightLog_t * flightLogCreate(int fd)
{
	const char *mapped;
	struct stat stats;
	size_t fileSize;

	const char *logSearchStart;
	int logIndex;

	flightLog_t *log;
	flightLogPrivate_t *private;

	// Map the log into memory
	if (fd < 0 || fstat(fd, &stats) < 0) {
		fprintf(stderr, "Failed to use log file: %s\n", strerror(errno));
		return 0;
	}

	fileSize = stats.st_size;

	if (fileSize == 0) {
		fprintf(stderr, "Error: This log is zero-bytes long!\n");
		return 0;
	}

#ifdef WIN32
	intptr_t fileHandle = _get_osfhandle(fd);
	HANDLE mapping = CreateFileMapping((HANDLE) fileHandle, NULL, PAGE_READONLY, 0, 0, NULL);

	if (mapping == NULL) {
		fprintf(stderr, "Failed to map log file into memory\n");
		return 0;
	}

	mapped = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, fileSize);

	if (mapped == NULL) {
		fprintf(stderr, "Failed to map log file into memory: %s\n", strerror(errno));

		return 0;
	}
#else
	mapped = mmap(0, fileSize, PROT_READ, MAP_PRIVATE, fd, 0);

	if (mapped == MAP_FAILED) {
		fprintf(stderr, "Failed to map log file into memory: %s\n", strerror(errno));

		return 0;
	}
#endif

	log = (flightLog_t *) malloc(sizeof(*log));
	private = (flightLogPrivate_t *) malloc(sizeof(*private));

	memset(log, 0, sizeof(*log));
	memset(private, 0, sizeof(*private));

    //First check how many logs are in this one file (each time the FC is rearmed, a new log is appended)
    logSearchStart = mapped;

    for (logIndex = 0; logIndex < FLIGHT_LOG_MAX_LOGS_IN_FILE && logSearchStart < mapped + fileSize; logIndex++) {
		log->logBegin[logIndex] = memmem(logSearchStart, (mapped + fileSize) - logSearchStart, LOG_START_MARKER, strlen(LOG_START_MARKER));

		if (!log->logBegin[logIndex])
			break; //No more logs found in the file

		//Search for the next log after this header ends
		logSearchStart = log->logBegin[logIndex] + strlen(LOG_START_MARKER);
	}

	log->logCount = logIndex;

	/*
	 * Stick the end of the file as the beginning of the "one past end" log, so we can easily compute each log size.
	 *
	 * We have room for this because the logBegin array has an extra element on the end for it.
	 */
	log->logBegin[log->logCount] = mapped + fileSize;

	private->logData = mapped;
    private->fd = fd;

	log->private = private;

    return log;
}

bool flightLogParse(flightLog_t *log, int logIndex, FlightLogMetadataReady onMetadataReady, FlightLogFrameReady onFrameReady, bool raw)
{
	ParserState parserState = PARSER_STATE_HEADER;
	bool mainStreamIsValid = false;

	char lastFrameType = 0;
	bool prematureEof = false;
	const char *frameStart = 0;

	flightLogPrivate_t *private = log->private;

	if (logIndex < 0 || logIndex >= log->logCount)
		return false;

	//Reset any parsed information from previous parses
	memset(&log->stats, 0, sizeof(log->stats));
	free(log->private->fieldNamesCombined);
	log->private->fieldNamesCombined = NULL;
	log->mainFieldCount = 0;
	log->gpsFieldCount = 0;

	//Default to MW's defaults
	log->minthrottle = 1150;
	log->maxthrottle = 1850;

	log->vbatref = 4095;
    log->vbatscale = 110;
	log->vbatmincellvoltage = 33;
	log->vbatmaxcellvoltage = 43;
    log->vbatwarningcellvoltage = 35;

	log->frameIntervalI = 32;
	log->frameIntervalPNum = 1;
	log->frameIntervalPDenom = 1;

	private->motor0Index = -1;

	//Set parsing ranges up for the log the caller selected
	private->logStart = log->logBegin[logIndex];
    private->logPos = private->logStart;
    private->logEnd = log->logBegin[logIndex + 1];
    private->eof = false;

	while (1) {
		int command = readChar(log);

		switch (parserState) {
			case PARSER_STATE_HEADER:
				switch (command) {
					case 'H':
						parseHeader(log);
					break;
					case 'I':
					case 'P':
					case 'G':
						unreadChar(log, command);

						if (log->mainFieldCount == 0) {
							fprintf(stderr, "Data file is missing field name definitions\n");
							return false;
						}

						parserState = PARSER_STATE_BEFORE_FIRST_FRAME;

						if (onMetadataReady)
							onMetadataReady(log);
					break;
					case EOF:
						fprintf(stderr, "Data file contained no events\n");
						return false;
				}
			break;
			case PARSER_STATE_BEFORE_FIRST_FRAME:
				lastFrameType = (char) command;
				frameStart = private->logPos;

				switch (command) {
					case 'I':
						parseIntraframe(log, raw);

						if (private->eof)
							prematureEof = true;
						else
							parserState = PARSER_STATE_DATA;
					break;
					case EOF:
						fprintf(stderr, "Data file contained no events\n");
						return false;
					break;
					default:
						//Ignore leading garbage
					break;
				}
			break;
			case PARSER_STATE_DATA:
				if (lastFrameType == 'P' || lastFrameType == 'I') {
					unsigned int lastFrameSize = private->logPos - frameStart;

					/*
					 * If we didn't reach the end of the log prematurely, and see what looks like the beginning of a new
					 * frame, assume that the previous frame was valid:
					 */
					if (!prematureEof && (command == 'I' || command == 'P' || command == 'G' || command == 'H' || command == EOF)) {
						if (lastFrameType == 'I' || lastFrameType == 'P') {
							if (lastFrameType == 'I') {
								updateFrameSizeStats(log->stats.iFrameSizeCount, lastFrameSize);

								// Only accept this frame as valid if time and iteration count are moving forward:
								if (raw || ((uint32_t)private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] >= log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_ITERATION]
									&& (uint32_t)private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_TIME] >= log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME]))
									mainStreamIsValid = true;

								log->stats.iFrameBytes += lastFrameSize;
								log->stats.numIFrames++;
							} else if (lastFrameType == 'P' && mainStreamIsValid) {
								updateFrameSizeStats(log->stats.pFrameSizeCount, lastFrameSize);

								log->stats.pFrameBytes += lastFrameSize;
								log->stats.numPFrames++;
								//Receiving a P frame can't resynchronise the stream so it doesn't set stateIsValid to true
							}

							if (mainStreamIsValid) {
								updateFieldStatistics(log, private->mainHistory[0]);
							} else {
								log->stats.numUnusablePFrames++;
							}

							if (onFrameReady)
								onFrameReady(log, mainStreamIsValid, private->mainHistory[0], lastFrameType, log->mainFieldCount, frameStart - private->logData, lastFrameSize);
						}
					} else {
						//Otherwise the previous frame was corrupt
						if (lastFrameType == 'I' || lastFrameType == 'P') {
							log->stats.numBrokenFrames++;

							//We need to resynchronise before we can deliver another frame:
							mainStreamIsValid = false;
						}

						//Let the caller know there was a corrupt frame (don't give them a pointer to the frame data because it is totally worthless)
						if (onFrameReady)
							onFrameReady(log, false, 0, lastFrameType, 0, frameStart - private->logData, lastFrameSize);

						/*
						 * Start the search for a frame beginning at the first byte of the previous, corrupt frame.
						 * This way we can find the start of the next frame after the corrupt frame
						 * if the corrupt frame was truncated.
						 */
						private->logPos = frameStart;
						lastFrameType = '\0';
						prematureEof = false;
						private->eof = false;
						continue;
					}
				}

				lastFrameType = (char) command;
				frameStart = private->logPos;

				switch (command) {
					case 'I':
						for (uint32_t frameIndex = log->private->mainHistory[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1; !shouldHaveFrame(log, frameIndex); frameIndex++) {
							log->stats.intentionallyAbsentFrames++;
						}

						parseIntraframe(log, raw);
					break;
					case 'P':
						parseInterframe(log, raw);
					break;
					case 'G':
						parseGPSFrame(log, raw);
					break;
					case 'H':
						parseGPSHomeFrame(log, raw);
					break;
					case EOF:
						goto done;
					default:
						mainStreamIsValid = false;
				}

				if (private->eof)
					prematureEof = true;
			break;
		}
	}

	done:
	log->stats.totalBytes = private->logEnd - private->logStart;

	return true;
}

void flightLogDestroy(flightLog_t *log)
{
#ifdef WIN32
	UnmapViewOfFile(log->private->logData);
#else
	munmap((void*)log->private->logData, log->private->logEnd - log->private->logData);
#endif

	free(log->private->fieldNamesCombined);
	free(log->private);
	free(log);

	//TODO clean up mainFieldNames
}
