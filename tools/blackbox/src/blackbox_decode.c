#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include <errno.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#include "parser.h"

int optionRaw = 0, optionStatsLimits = 0, optionDebug;
int optionLogNumber = -1;
const char *optionFilename = 0;

uint32_t lastFrameIndex = (uint32_t) -1;

void onFrameReady(flightLog_t *log, int32_t *frame, int frameOffset, int frameSize)
{
	int i;

	if (optionDebug) {
		lastFrameIndex++;

		while (lastFrameIndex != (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION]) {
			fprintf(stderr, "Missing frame %u\n", lastFrameIndex);
			lastFrameIndex++;
		}

		if (frame[FLIGHT_LOG_FIELD_INDEX_ITERATION] % 32 == 0)
			printf("I ");
		else
			printf("P ");
	}

	for (i = 0; i < log->fieldCount; i++) {
		if (i > 0)
			printf(", ");

		printf("%3d", frame[i]);
	}

	if (optionDebug)
		printf(", at %d, size %d", frameOffset, frameSize);

	printf("\n");
}

void onMetadataReady(flightLog_t *log)
{
	int i;

	for (i = 0; i < log->fieldCount; i++) {
		if (i > 0)
			printf(", ");

		printf("%s", log->fieldNames[i]);
	}
	printf("\n");
}

void printStats(flightLog_t *log, bool raw, bool limits)
{
	flightLogStatistics_t *stats = &log->stats;
	uint32_t intervalMS = (stats->fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] - stats->fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]) / 1000;

	uint32_t goodBytes = stats->iFrameBytes + stats->pFrameBytes;
	uint32_t goodFrames = stats->numIFrames + stats->numPFrames;
	uint32_t totalFrames = stats->numIFrames + stats->numPFrames + stats->numBrokenFrames;

	fprintf(stderr, "\nStatistics\n");

	if (stats->numIFrames)
		fprintf(stderr, "I frames %7d %6d bytes avg %8d bytes total\n", stats->numIFrames, stats->iFrameBytes / stats->numIFrames, stats->iFrameBytes);

	if (stats->numPFrames)
		fprintf(stderr, "P frames %7d %6d bytes avg %8d bytes total\n", stats->numPFrames, stats->pFrameBytes / stats->numPFrames, stats->pFrameBytes);

	if (goodFrames)
		fprintf(stderr, "Frames %9d %6d bytes avg %8d bytes total\n", goodFrames, goodBytes / goodFrames, goodBytes);
	else
		fprintf(stderr, "Frames %8d\n", 0);

	if (stats->numBrokenFrames)
		fprintf(stderr, "%d frames failed to decode (%.2f%%)\n", stats->numBrokenFrames, (double) stats->numBrokenFrames / totalFrames * 100);

	if (intervalMS > 0 && !raw) {
		fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
				(unsigned int) (((int64_t) totalFrames * 1000) / intervalMS),
				(unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
				(unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
	} else {
		fprintf(stderr, "Data rate: Unknown, no timing information available.\n");
	}

	if (limits) {
		fprintf(stderr, "\n\n    Field name          Min          Max        Range\n");
		fprintf(stderr,     "-----------------------------------------------------\n");

		for (int i = 0; i < log->fieldCount; i++) {
			fprintf(stderr, "%14s %12" PRId64 " %12" PRId64 " %12" PRId64 "\n",
				log->fieldNames[i],
				stats->fieldMinimum[i],
				stats->fieldMaximum[i],
				stats->fieldMaximum[i] - stats->fieldMinimum[i]
			);
		}
	}
}

int onChooseLog(int logCount, const char **logStarts)
{
	if (logCount == 0) {
		fprintf(stderr, "Couldn't find the header of a flight log in this file, is this the right kind of file?\n");
		return -1;
	}

	//Did the user pick a log to render?
	if (optionLogNumber > 0) {
		if (optionLogNumber > logCount) {
			fprintf(stderr, "Couldn't load log #%d from this file, because there are only %d logs in total.\n", optionLogNumber, logCount);
			return -1;
		}

		return optionLogNumber - 1;
	} else if (logCount == 1) {
		// If there's only one log, just parse that
		return 0;
	} else {
		fprintf(stderr, "This file contains multiple flight logs, please choose one with the --index argument:\n\n");

		fprintf(stderr, "Index  Start offset  Size (bytes)\n");
		for (int i = 0; i < logCount; i++) {
			fprintf(stderr, "%5d %13d %13d\n", i + 1, (int) (logStarts[i] - logStarts[0]), (int) (logStarts[i + 1] - logStarts[i]));
		}

		return -1;
	}
}

void parseCommandlineOptions(int argc, char **argv)
{
	int c;

	while (1)
	{
		static struct option long_options[] = {
			{"raw", no_argument, &optionRaw, 1},
			{"debug", no_argument, &optionDebug, 1},
			{"limits", no_argument, &optionStatsLimits, 1},
			{"index", required_argument, 0, 'i'},
			{0, 0, 0, 0}
		};

		int option_index = 0;

		c = getopt_long (argc, argv, "", long_options, &option_index);

		if (c == -1)
			break;

		switch (c) {
			case 'i':
				optionLogNumber = atoi(optarg);
			break;
		}
	}

	if (optind < argc)
		optionFilename = argv[optind];
}

int main(int argc, char **argv)
{
	flightLog_t *log;
	int fd;

	parseCommandlineOptions(argc, argv);

	if (!optionFilename) {
		fprintf(stderr, "Missing log filename argument\n");
		return -1;
	}

    fd = open(optionFilename, O_RDONLY);
    if (fd < 0) {
    	fprintf(stderr, "Failed to open log file '%s': %s\n", optionFilename, strerror(errno));
    	return -1;
    }

	log = parseFlightLog(fd, onChooseLog, onMetadataReady, onFrameReady, optionRaw);

	if (log) {
		printStats(log, optionRaw, optionStatsLimits);

		// destroyFlightLog(log); Leaking it is faster.
	} else {
		return -1;
	}

	return 0;
}
