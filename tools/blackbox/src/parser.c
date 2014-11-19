#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/mman.h>
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

	int motor0Index;

	int32_t blackboxHistoryRing[2][FLIGHT_LOG_MAX_FIELDS];
	int32_t* blackboxHistory[2];

	int fd;

	//The start of the entire log file:
	const char *logData;

	//The section of the file which is currently being examined:
	const char *logStart, *logEnd, *logPos;
} flightLogPrivate_t;

static int readChar(flightLog_t *log)
{
	if (log->private->logPos < log->private->logEnd) {
		int result = (uint8_t) *log->private->logPos;
		log->private->logPos++;
		return result;
	}

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

		log->fieldNames[log->fieldCount++] = start;

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

	if (strcmp(fieldName, "Field name") == 0) {
		parseFieldNames(log, fieldValue);

		for (i = 0; i < log->fieldCount; i++) {
			if (strcmp(log->fieldNames[i], "motor[0]") == 0) {
				log->private->motor0Index = i;
				break;
			}
		}
	} else if (strcmp(fieldName, "Field P-predictor") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldPPredictor, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field P-encoding") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldPEncoding, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field I-predictor") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldIPredictor, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field I-encoding") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->private->fieldIEncoding, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "Field signed") == 0) {
		parseCommaSeparatedIntegers(fieldValue, log->fieldSigned, FLIGHT_LOG_MAX_FIELDS);
	} else if (strcmp(fieldName, "minthrottle") == 0) {
		log->minthrottle = atoi(fieldValue);
	} else if (strcmp(fieldName, "rcRate") == 0) {
		log->rcRate = atoi(fieldValue);
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
	return (i >> 1) ^ (-(i & 1));
}


static int32_t signExtend4Bit(uint8_t nibble)
{
	return (nibble & 0x08) ? (int32_t) (int8_t) (nibble | 0xF0) : nibble;
}

static void readTag8_4S16(flightLog_t *log, int32_t *values) {
	uint8_t selector, combinedChar;
	uint8_t char1, char2;
	int i;

	selector = readChar(log);

	//Read the 4 values from the stream
	for (i = 0; i < 4; i++) {
		switch (selector & 0x03) {
			case 0: // Zero
				values[i] = 0;
			break;
			case 1: // Two 4-bit fields
				combinedChar = (uint8_t) readChar(log);

				values[i] = signExtend4Bit(combinedChar & 0x0F);

				i++;
				selector >>= 2;

				values[i] = signExtend4Bit(combinedChar >> 4);
			break;
			case 2: // 8-bit field
				//Sign extend...
				values[i] = (int32_t) (int8_t) readChar(log);
			break;
			case 3: // 16-bit field
				char1 = readChar(log);
				char2 = readChar(log);

				//Sign extend...
				values[i] = (int16_t) (char1 | (char2 << 8));
			break;
		}

		selector >>= 2;
	}
}

static void parseIntraframe(flightLog_t *log, bool raw)
{
	int i;

	log->private->blackboxHistory[0] = &log->private->blackboxHistoryRing[0][0];
	log->private->blackboxHistory[1] = &log->private->blackboxHistoryRing[0][0];

	for (i = 0; i < log->fieldCount; i++) {
		uint32_t value;

		switch (log->private->fieldIEncoding[i]) {
			case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
				value = (uint32_t) readSignedVB(log);
			break;
			case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
				value = readUnsignedVB(log);
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
				case FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0:
					if (log->private->motor0Index < 0) {
						fprintf(stderr, "Attempted to base I-field prediction on motor0 before it was read\n");
						exit(-1);
					}
					value += (uint32_t) log->private->blackboxHistory[0][log->private->motor0Index];
				break;
				default:
					fprintf(stderr, "Unsupported I-field predictor %d\n", log->private->fieldIPredictor[i]);
					exit(-1);
			}
		}

		log->private->blackboxHistory[0][i] = (int32_t) value;
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
		case FLIGHT_LOG_FIELD_PREDICTOR_1:
			value += (uint32_t) log->private->blackboxHistory[0][fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE:
			value += 2 * (uint32_t) log->private->blackboxHistory[0][fieldIndex] - (uint32_t) log->private->blackboxHistory[1][fieldIndex];
		break;
		case FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2:
			if (log->fieldSigned[fieldIndex])
				value += (uint32_t) ((int32_t) ((uint32_t) log->private->blackboxHistory[0][fieldIndex] + (uint32_t) log->private->blackboxHistory[1][fieldIndex]) / 2);
			else
				value += ((uint32_t) log->private->blackboxHistory[0][fieldIndex] + (uint32_t) log->private->blackboxHistory[1][fieldIndex]) / 2;
		break;
		default:
			fprintf(stderr, "Unsupported P-field predictor %d\n", log->private->fieldPPredictor[fieldIndex]);
			exit(-1);
	}

	return (int32_t) value;
}

static void parseInterframe(flightLog_t *log, bool raw)
{
	int i;

	// The new frame gets stored over the top of the current oldest history
	int32_t *newFrame = log->private->blackboxHistory[0] == &log->private->blackboxHistoryRing[0][0] ? &log->private->blackboxHistoryRing[1][0] : &log->private->blackboxHistoryRing[0][0];

	for (i = 0; i < log->fieldCount; i++) {
		uint32_t value;
		uint32_t values[4];

		if (log->private->fieldPPredictor[i] == FLIGHT_LOG_FIELD_PREDICTOR_INC) {
			newFrame[i] = log->private->blackboxHistory[0][i] + 1;
		} else {
			switch (log->private->fieldPEncoding[i]) {
				case FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB:
					value = (uint32_t) readSignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB:
					value = readUnsignedVB(log);
				break;
				case FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16:
					readTag8_4S16(log, (int32_t*)values);

					//Apply the predictors for the fields:
					for (int j = 0; j < 3; j++) {
						// ^ But don't process the final value, allow the regular handler after the 'case' to do that
						newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], values[j]);
						i++;
					}
					value = values[3];
				break;
				default:
					fprintf(stderr, "Unsupported P-field encoding %d\n", log->private->fieldPEncoding[i]);
					exit(-1);
			}

			newFrame[i] = applyInterPrediction(log, i, raw ? FLIGHT_LOG_FIELD_PREDICTOR_0 : log->private->fieldPPredictor[i], value);
		}
	}

	log->private->blackboxHistory[1] = log->private->blackboxHistory[0];
	log->private->blackboxHistory[0] = newFrame;
}

static void updateFieldStatistics(flightLog_t *log, int32_t *fields)
{
	int i;

	if (log->stats.numIFrames + log->stats.numPFrames <= 1) {
		//If this is the first frame, there are no minimums or maximums in the stats to compare with
		for (i = 0; i < log->fieldCount; i++) {
			if (log->fieldSigned[i]) {
				log->stats.fieldMaximum[i] = fields[i];
				log->stats.fieldMinimum[i] = fields[i];
			} else {
				log->stats.fieldMaximum[i] = (uint32_t) fields[i];
				log->stats.fieldMinimum[i] = (uint32_t) fields[i];
			}
		}
	} else {
		for (i = 0; i < log->fieldCount; i++) {
			if (log->fieldSigned[i]) {
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

	mapped = mmap(0, fileSize, PROT_READ, MAP_PRIVATE, fd, 0);

	if (mapped == MAP_FAILED) {
		fprintf(stderr, "Failed to map log file into memory: %s\n", strerror(errno));

		return 0;
	}

	log = (flightLog_t *) malloc(sizeof(flightLog_t));
	private = (flightLogPrivate_t *) malloc(sizeof(flightLogPrivate_t));

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
	bool streamIsValid = false;

	char lastFrameType = 0;
	const char *frameStart = 0;

	flightLogPrivate_t *private = log->private;

	if (logIndex < 0 || logIndex >= log->logCount)
		return false;

	//Reset any parsed information from previous parses
	memset(&log->stats, 0, sizeof(log->stats));
	free(log->private->fieldNamesCombined);
	log->private->fieldNamesCombined = NULL;
	log->fieldCount = 0;
	private->motor0Index = -1;

	//Set parsing ranges up for the log the caller selected
	private->logStart = log->logBegin[logIndex];
    private->logPos = private->logStart;
    private->logEnd = log->logBegin[logIndex + 1];

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
						unreadChar(log, command);

						if (log->fieldCount == 0) {
							fprintf(stderr, "Data file is missing field name definitions\n");
							return false;
						}

						parserState = PARSER_STATE_BEFORE_FIRST_FRAME;

						if (onMetadataReady)
							onMetadataReady(log);
					break;
					case EOF:
						fprintf(stderr, "Data file contained no events\n");
						return 0;
				}
			break;
			case PARSER_STATE_BEFORE_FIRST_FRAME:
				lastFrameType = (char) command;
				frameStart = private->logPos;

				switch (command) {
					case 'I':
						parseIntraframe(log, raw);
						parserState = PARSER_STATE_DATA;
					break;
					default:
						//Ignore leading garbage
					break;
				}
			break;
			case PARSER_STATE_DATA:
				if (lastFrameType == 'P' || lastFrameType == 'I') {

					//If we see what looks like the beginning of a new frame, assume that the previous frame was valid:
					if (command == 'I' || command == 'P' || command == EOF) {
						unsigned int lastFrameSize = private->logPos - frameStart;

						if (lastFrameType == 'I') {
							updateFrameSizeStats(log->stats.iFrameSizeCount, lastFrameSize);

							// Only accept this frame as valid if time and iteration count are moving forward:
							if (private->blackboxHistory[0][FLIGHT_LOG_FIELD_INDEX_ITERATION] >= log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_ITERATION]
								&& private->blackboxHistory[0][FLIGHT_LOG_FIELD_INDEX_TIME] >= log->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME])
								streamIsValid = true;

							log->stats.iFrameBytes += lastFrameSize;
							log->stats.numIFrames++;
						} else if (lastFrameType == 'P' && streamIsValid) {
							updateFrameSizeStats(log->stats.pFrameSizeCount, lastFrameSize);

							log->stats.pFrameBytes += lastFrameSize;
							log->stats.numPFrames++;
							//Receiving a P frame can't resynchronise the stream so it doesn't set stateIsValid to true
						}

						if (streamIsValid) {
							updateFieldStatistics(log, private->blackboxHistory[0]);

							if (onFrameReady)
								onFrameReady(log, private->blackboxHistory[0], frameStart - private->logData, lastFrameSize);
						}
					} else {
						//Otherwise the previous frame was corrupt
						log->stats.numBrokenFrames++;

						//We need to resynchronise before we can deliver another frame:
						streamIsValid = false;
					}
				}

				lastFrameType = (char) command;
				frameStart = private->logPos;

				switch (command) {
					case 'I':
						parseIntraframe(log, raw);
					break;
					case 'P':
						parseInterframe(log, raw);
					break;
					case EOF:
						goto done;
					default:
						streamIsValid = false;
				}
			break;
		}
	}

	done:
	log->stats.totalBytes = private->logEnd - private->logStart;

	return true;
}

void flightLogDestroy(flightLog_t *log)
{
	munmap((void*)log->private->logData, log->private->logEnd - log->private->logData);

	free(log->private->fieldNamesCombined);
	free(log->private);
	free(log);

	//TODO clean up fieldNames
}
