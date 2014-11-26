#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include <errno.h>
#include <fcntl.h>

#ifdef WIN32
	#include <io.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef WIN32
	#include "getopt.h"
#else
	#include <getopt.h>
#endif

#include "parser.h"

typedef struct decodeOptions_t {
	int help, raw, limits, debug;
	int logNumber;
	const char *filename;
} decodeOptions_t;

decodeOptions_t options = {
	.help = 0, .raw = 0, .limits = 0, .debug = 0,
	.logNumber = -1,
	.filename = 0
};

uint32_t lastFrameIndex = (uint32_t) -1;

void onFrameReady(flightLog_t *log, bool frameValid, int32_t *frame, int frameOffset, int frameSize)
{
	int i;

	if (frame)
		lastFrameIndex = (uint32_t) frame[FLIGHT_LOG_FIELD_INDEX_ITERATION];

	if (frameValid) {
		for (i = 0; i < log->fieldCount; i++) {
			if (i == 0) {
				printf("%u", (uint32_t) frame[i]);
			} else {
				if (log->fieldSigned[i] || options.raw)
					printf(", %3d", frame[i]);
				else
					printf(", %3u", (uint32_t) frame[i]);
			}
		}

		if (options.debug) {
			printf(", %c, offset %d, size %d\n", frame[FLIGHT_LOG_FIELD_INDEX_ITERATION] % 32 == 0 ? 'I' : 'P', frameOffset, frameSize);
		} else
			printf("\n");
	} else if (options.debug) {
		if (frame) {
			/*
			 * We'll assume that the frame's iteration count is still fairly sensible (if an earlier frame was corrupt,
			 * the frame index will be smaller than it should be)
			 */
			fprintf(stderr, "Frame unusuable due to prior corruption %u, offset %d, size %d\n", lastFrameIndex, frameOffset, frameSize);
		} else {
			//We have no frame index for this frame, so just assume it was the one after the previously decoded frame
			lastFrameIndex++;
			fprintf(stderr, "Failed to decode frame %u, offset %d, size %d\n", lastFrameIndex, frameOffset, frameSize);
		}
	}
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

void printStats(flightLog_t *log, int logIndex, bool raw, bool limits)
{
	flightLogStatistics_t *stats = &log->stats;
	uint32_t intervalMS = (uint32_t) ((stats->fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] - stats->fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]) / 1000);

	uint32_t goodBytes = stats->iFrameBytes + stats->pFrameBytes;
	uint32_t goodFrames = stats->numIFrames + stats->numPFrames;
	uint32_t totalFrames = (uint32_t) (stats->fieldMaximum[FLIGHT_LOG_FIELD_INDEX_ITERATION] - stats->fieldMinimum[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1);
	uint32_t missingFrames = totalFrames - goodFrames;

	uint32_t runningTimeMS, runningTimeSecs, runningTimeMins;
	uint32_t startTimeMS, startTimeSecs, startTimeMins;
	uint32_t endTimeMS, endTimeSecs, endTimeMins;

	runningTimeMS = intervalMS;
	runningTimeSecs = runningTimeMS / 1000;
	runningTimeMS %= 1000;
	runningTimeMins = runningTimeSecs / 60;
	runningTimeSecs %= 60;

	startTimeMS = (uint32_t) (stats->fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME] / 1000);
	startTimeSecs = startTimeMS / 1000;
	startTimeMS %= 1000;
	startTimeMins = startTimeSecs / 60;
	startTimeSecs %= 60;

	endTimeMS = (uint32_t) (stats->fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME] / 1000);
	endTimeSecs = endTimeMS / 1000;
	endTimeMS %= 1000;
	endTimeMins = endTimeSecs / 60;
	endTimeSecs %= 60;

	fprintf(stderr, "\nLog #%d/%d, start %02d:%02d.%03d, end %02d:%02d.%03d, duration %02d:%02d.%03d\n\n", logIndex + 1, log->logCount,
		startTimeMins, startTimeSecs, startTimeMS,
		endTimeMins, endTimeSecs, endTimeMS,
		runningTimeMins, runningTimeSecs, runningTimeMS
	);

	fprintf(stderr, "Statistics\n");

	if (stats->numIFrames)
		fprintf(stderr, "I frames %7d %6.1f bytes avg %8d bytes total\n", stats->numIFrames, (float) stats->iFrameBytes / stats->numIFrames, stats->iFrameBytes);

	if (stats->numPFrames)
		fprintf(stderr, "P frames %7d %6.1f bytes avg %8d bytes total\n", stats->numPFrames, (float) stats->pFrameBytes / stats->numPFrames, stats->pFrameBytes);

	if (goodFrames)
		fprintf(stderr, "Frames %9d %6.1f bytes avg %8d bytes total\n", goodFrames, (float) goodBytes / goodFrames, goodBytes);
	else
		fprintf(stderr, "Frames %8d\n", 0);

	if (intervalMS > 0 && !raw) {
		fprintf(stderr, "Data rate %4uHz %6u bytes/s %10u baud\n",
				(unsigned int) (((int64_t) totalFrames * 1000) / intervalMS),
				(unsigned int) (((int64_t) stats->totalBytes * 1000) / intervalMS),
				(unsigned int) ((((int64_t) stats->totalBytes * 1000 * 8) / intervalMS + 100 - 1) / 100 * 100)); /* Round baud rate up to nearest 100 */
	} else {
		fprintf(stderr, "Data rate: Unknown, no timing information available.\n");
	}

	if (totalFrames && (stats->numBrokenFrames || stats->numUnusablePFrames || missingFrames)) {
		fprintf(stderr, "\n");

		if (stats->numBrokenFrames || stats->numUnusablePFrames) {
			fprintf(stderr, "%d frames failed to decode, rendering %d P-frames unusable. ", stats->numBrokenFrames, stats->numUnusablePFrames);
			if (!missingFrames)
				fprintf(stderr, "\n");
		}
		if (missingFrames) {
			fprintf(stderr, "%d frames are missing in total (%ums, %.2f%%)\n",
				missingFrames,
				(unsigned int) (missingFrames * (intervalMS / totalFrames)), (double) missingFrames / totalFrames * 100);
		}
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

int validateLogIndex(flightLog_t *log)
{
	if (log->logCount == 0) {
		fprintf(stderr, "Couldn't find the header of a flight log in this file, is this the right kind of file?\n");
		return -1;
	}

	//Did the user pick a log to render?
	if (options.logNumber > 0) {
		if (options.logNumber > log->logCount) {
			fprintf(stderr, "Couldn't load log #%d from this file, because there are only %d logs in total.\n", options.logNumber, log->logCount);
			return -1;
		}

		return options.logNumber - 1;
	} else if (log->logCount == 1) {
		// If there's only one log, just parse that
		return 0;
	} else {
		fprintf(stderr, "This file contains multiple flight logs, please choose one with the --index argument:\n\n");

		fprintf(stderr, "Index  Start offset  Size (bytes)\n");
		for (int i = 0; i < log->logCount; i++) {
			fprintf(stderr, "%5d %13d %13d\n", i + 1, (int) (log->logBegin[i] - log->logBegin[0]), (int) (log->logBegin[i + 1] - log->logBegin[i]));
		}

		return -1;
	}
}

void printUsage(const char *argv0)
{
	fprintf(stderr,
		"Blackbox flight log decoder by Nicholas Sherlock\n\n"
		"Usage:\n"
		"     %s [options] <logfilename.txt>\n\n"
		"Options:\n"
		"   --help         This page\n"
		"   --index <num>  Choose the log from the file that should be decoded\n"
		"   --limits       Print the limits and range of each field\n"
		"   --debug        Show extra debugging information\n"
		"   --raw          Don't apply predictions to fields (show raw field deltas)\n"
		"\n", argv0
	);
}

void parseCommandlineOptions(int argc, char **argv)
{
	int c;

	while (1)
	{
		static struct option long_options[] = {
			{"help", no_argument, &options.help, 1},
			{"raw", no_argument, &options.raw, 1},
			{"debug", no_argument, &options.debug, 1},
			{"limits", no_argument, &options.limits, 1},
			{"index", required_argument, 0, 'i'},
			{0, 0, 0, 0}
		};

		int option_index = 0;

		c = getopt_long (argc, argv, "", long_options, &option_index);

		if (c == -1)
			break;

		switch (c) {
			case 'i':
				options.logNumber = atoi(optarg);
			break;
		}
	}

	if (optind < argc)
		options.filename = argv[optind];
}

int main(int argc, char **argv)
{
	flightLog_t *log;
	int fd;
	int logIndex;

	parseCommandlineOptions(argc, argv);

	if (options.help || !options.filename) {
		printUsage(argv[0]);
		return -1;
	}

    fd = open(options.filename, O_RDONLY);
    if (fd < 0) {
    	fprintf(stderr, "Failed to open log file '%s': %s\n", options.filename, strerror(errno));
    	return -1;
    }

    log = flightLogCreate(fd);

    if (!log)
    	return -1;

    logIndex = validateLogIndex(log);

    if (logIndex == -1)
    	return -1;

	if (flightLogParse(log, logIndex, onMetadataReady, onFrameReady, options.raw)) {
		printStats(log, logIndex, options.raw, options.limits);

		// destroyFlightLog(log); Leaking it is faster.
	} else {
		return -1;
	}

	return 0;
}
