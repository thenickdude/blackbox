#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <errno.h>

#include <unistd.h>
#include <fcntl.h>

#include <pthread.h>

#ifdef __APPLE__
//MacOS doesn't have POSIX unnamed semaphores. Grand Central Dispatch provides an alternative:
#include <dispatch/dispatch.h>
#else
#include <semaphore.h>
#endif

#include <cairo.h>
#include <png.h>

#include "parser.h"
#include "datapoints.h"

#define MAX_MOTORS 8

//Controls how fast the props spin on the video
#define MOTOR_MAX_RPS 25

//TODO read me from flight log
#define MINTHROTTLE 1150
#define MAXTHROTTLE 1850

#define PNG_RENDERING_THREADS 3

typedef enum PropStyle {
	PROP_STYLE_BLADES = 0,
	PROP_STYLE_PIE_CHART = 1
} PropStyle;

typedef struct color_t {
	double r, g, b;
} color_t;

typedef struct colorAlpha_t {
	double r, g, b, a;
} colorAlpha_t;

typedef struct pngRenderingTask_t {
	cairo_surface_t *surface;
	int outputFrameIndex;
} pngRenderingTask_t;

typedef struct craftDrawingParameters_t {
	int numBlades, numMotors;
	int bladeLength;
	double tipBezierWidth, tipBezierHeight;
	double motorSpacing;

	double motorX[MAX_MOTORS];
	double motorY[MAX_MOTORS];
	int motorDirection[MAX_MOTORS];

	color_t propColor[MAX_MOTORS];
} craft_parameters_t;

typedef struct renderOptions_t {
	int logNumber;
	int imageWidth, imageHeight;
	int fps;
	int help;

	bool plotPids, plotPidSum, plotGyros, plotMotors;
	bool drawPidTable, drawSticks, drawCraft;

	int pidSmoothing, gyroSmoothing, motorSmoothing;

	PropStyle propStyle;

	uint32_t timeStart, timeEnd;

	char *filename, *outputPrefix;
} renderOptions_t;

const double DASHED_LINE[] = {
	20.0,  /* ink */
	5.0  /* skip */
};
const int DASHED_LINE_NUM_POINTS = sizeof (DASHED_LINE) / sizeof(DASHED_LINE[0]);

const double DOTTED_LINE[] = {
	5,  /* ink */
	5  /* skip */
};
const int DOTTED_LINE_NUM_POINTS = sizeof (DOTTED_LINE) / sizeof(DOTTED_LINE[0]);

#define PID_P 0
#define PID_I 1
#define PID_D 2
#define PID_TOTAL 3

typedef struct fieldIdentifications_t {
	int rcCommandFields[4];

	int numMotors;
	int motorFields[MAX_MOTORS];
	color_t motorColors[MAX_MOTORS];

	bool hasPIDs;
	int axisPIDFields[3][3]; //First dimension is [P, I, D], second dimension is axis
	color_t PIDAxisColors[3][3]; //PID, axis
	int PIDLineStyle[3]; //Indexed by PID_P etc

	bool hasGyros;
	int gyroFields[3];
	color_t gyroColors[3];

	int numMisc;
	int miscFields[FLIGHT_LOG_MAX_FIELDS];
	color_t miscColors[FLIGHT_LOG_MAX_FIELDS];
} fieldIdentifications_t;

color_t lineColors[] = {
	{0.553,	0.827,	0.78},
	{1,	1,	0.702},
	{0.745,	0.729,	0.855},
	{0.984,	0.502,	0.447},
	{0.502,	0.694,	0.827},
	{0.992,	0.706,	0.384},
	{0.702,	0.871,	0.412},
	{0.988,	0.804,	0.898},
	{0.851,	0.851,	0.851},
	{0.737,	0.502,	0.741},
	{0.8,	0.922,	0.773},
	{1,	0.929,	0.435}
};

#define NUM_LINE_COLORS (sizeof(lineColors) / sizeof(lineColors[0]))

const colorAlpha_t stickColor = {1, 0.4, 0.4, 1.0};
const colorAlpha_t stickAreaColor = {0.3, 0.3, 0.3, 0.8};
const colorAlpha_t craftColor = {0.3, 0.3, 0.3, 1};
const colorAlpha_t crosshairColor = {0.75, 0.75, 0.75, 0.5};

static const renderOptions_t defaultOptions = {
	.imageWidth = 1920, .imageHeight = 1080,
	.fps = 30, .help = 0, .propStyle = PROP_STYLE_PIE_CHART,
	.plotPids = false, .plotPidSum = false, .plotGyros = false, .plotMotors = true,
	.pidSmoothing = 4, .gyroSmoothing = 1, .motorSmoothing = 2,
	.drawCraft = true, .drawPidTable = true, .drawSticks = true,
	.filename = 0,
	.timeStart = 0, .timeEnd = 0,
	.logNumber = 0
};

static renderOptions_t options;

#ifdef __APPLE__
static dispatch_semaphore_t pngRenderingSem;
#else
static sem_t pngRenderingSem;
#endif

static flightLog_t *flightLog;
static Datapoints *points;

//Information about fields we have classified
static fieldIdentifications_t idents;

static double doubleAbs(double a)
{
	if (a < 0)
		return -a;
	return a;
}

static double doubleMin(double a, double b)
{
	if (a < b)
		return a;
	return b;
}

static double doubleMax(double a, double b)
{
	if (a > b)
		return a;
	return b;
}

static int32_t minS32(int32_t a, int32_t b) {
	if (a < b)
		return a;
	return b;
}

static int32_t maxS32(int32_t a, int32_t b) {
	if (a > b)
		return a;
	return b;
}

void loadFrameIntoPoints(flightLog_t *log, bool frameValid, int32_t *frame, int frameOffset, int frameSize)
{
	(void) log;
	(void) frameSize;
	(void) frameOffset;

	/*
	 *  Pull the first two fields (iteration and time) off the front of the frame fields,
	 * since datapoints handles those as separate arguments in this call:
	 */
	if (frameValid)
		datapointsSetFrame(points, frame[FLIGHT_LOG_FIELD_INDEX_ITERATION], frame[FLIGHT_LOG_FIELD_INDEX_TIME], frame + 2);
}

void identifyFields()
{
	unsigned int i;
	int fieldIndex;

	//Start off all the fields as -1 so we can use it as a not-present identifier
	for (i = 0; i < sizeof(idents.rcCommandFields) / sizeof(idents.rcCommandFields[0]); i++)
		idents.rcCommandFields[i] = -1;

	for (i = 0; i < sizeof(idents.motorFields) / sizeof(idents.motorFields[0]); i++)
		idents.motorFields[i] = -1;

	for (int pidType = PID_P; pidType <= PID_D; pidType++)
		for (int axis = 0; axis < 3; axis++)
			idents.axisPIDFields[pidType][axis] = -1;

	idents.hasGyros = false;
	idents.hasPIDs = false;

	for (i = 0; i < sizeof(idents.miscFields) / sizeof(idents.miscFields[0]); i++)
		idents.miscFields[i] = -1;

	//Now look through the field names and assign fields we recognize to each of those categories
	for (fieldIndex = 0; fieldIndex < points->fieldCount; fieldIndex++) {
		if (strncmp(points->fieldNames[fieldIndex], "motor[", strlen("motor[")) == 0) {
			int motorIndex = atoi(points->fieldNames[fieldIndex] + strlen("motor["));

			if (motorIndex >= 0 && motorIndex < MAX_MOTORS) {
				idents.motorFields[motorIndex] = fieldIndex;
				idents.motorColors[motorIndex] = lineColors[motorIndex % NUM_LINE_COLORS];
				idents.numMotors++;
			}
		} else if (strncmp(points->fieldNames[fieldIndex], "rcCommand[", strlen("rcCommand[")) == 0){
			int rcCommandIndex = atoi(points->fieldNames[fieldIndex] + strlen("rcCommand["));

			if (rcCommandIndex >= 0 && rcCommandIndex < 4) {
				idents.rcCommandFields[rcCommandIndex] = fieldIndex;
			}
		} else if (strncmp(points->fieldNames[fieldIndex], "axis", strlen("axis")) == 0) {
			int axisIndex = atoi(points->fieldNames[fieldIndex] + strlen("axisX["));

			switch (points->fieldNames[fieldIndex][strlen("axis")]) {
				case 'P':
					idents.axisPIDFields[PID_P][axisIndex] = fieldIndex;
				break;
				case 'I':
					idents.axisPIDFields[PID_I][axisIndex] = fieldIndex;
				break;
				case 'D':
					idents.axisPIDFields[PID_D][axisIndex] = fieldIndex;
				break;
			}

			idents.hasPIDs = true;

			idents.PIDAxisColors[PID_P][axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];

			idents.PIDAxisColors[PID_P][axisIndex].r *= 1.1;
			idents.PIDAxisColors[PID_P][axisIndex].g *= 1.1;
			idents.PIDAxisColors[PID_P][axisIndex].b *= 1.1;

			idents.PIDAxisColors[PID_I][axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];

			idents.PIDAxisColors[PID_D][axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];

			idents.PIDAxisColors[PID_D][axisIndex].r *= 0.9;
			idents.PIDAxisColors[PID_D][axisIndex].g *= 0.9;
			idents.PIDAxisColors[PID_D][axisIndex].b *= 0.9;

			idents.PIDLineStyle[axisIndex] = 0; //TODO
		} else if(strncmp(points->fieldNames[fieldIndex], "gyroData[", strlen("gyroData[")) == 0) {
			int axisIndex = atoi(points->fieldNames[fieldIndex] + strlen("gyroData["));

			idents.hasGyros = true;
			idents.gyroFields[axisIndex] = fieldIndex;
			idents.gyroColors[axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];
		} else {
			idents.miscFields[idents.numMisc] = fieldIndex;
			idents.miscColors[idents.numMisc] = lineColors[idents.numMisc % NUM_LINE_COLORS];
			idents.numMisc++;
		}
	}
}

void drawCommandSticks(int32_t *frame, int imageWidth, int imageHeight, cairo_t *cr)
{
	double rcCommand[4] = {0, 0, 0, 0};
	const int stickSurroundRadius = imageHeight / 11, stickSpacing = stickSurroundRadius * 3;
	const int pitchStickMax = 250 /*500 * (flightLog->rcRate ? flightLog->rcRate : 100) / 100*/,
			yawStickMax = 500;
	int stickIndex;

	char stickLabel[16];
	cairo_text_extents_t extent;

	(void) imageWidth;

	for (stickIndex = 0; stickIndex < 4; stickIndex++) {
		//Check that stick data is present to be drawn:
		if (idents.rcCommandFields[stickIndex] < 0)
			return;

		rcCommand[stickIndex] = frame[idents.rcCommandFields[stickIndex]];
	}

	//Compute the position of the sticks in the range [0..1] (left stick x, left stick y, right stick x, right stick y)
	double stickPositions[4];

	stickPositions[0] = -rcCommand[2] / (yawStickMax * 2) + 0.5;
	stickPositions[1] = (2000 - rcCommand[3]) / 1000;
	stickPositions[2] = rcCommand[0] / (pitchStickMax * 2) + 0.5;
	stickPositions[3] = (pitchStickMax - rcCommand[1]) / (pitchStickMax * 2);

	for (stickIndex = 0; stickIndex < 4; stickIndex++) {
		//Clamp to [0..1]
		stickPositions[stickIndex] = stickPositions[stickIndex] > 1 ? 1 : (stickPositions[stickIndex] < 0 ? 0 : stickPositions[stickIndex]);

		//Shift to [-0.5..0.5]
		stickPositions[stickIndex] -= 0.5;

		//Scale to our stick size
		stickPositions[stickIndex] *= stickSurroundRadius * 2;
	}

	cairo_save(cr);

	cairo_translate(cr, -stickSpacing / 2, 0);

	//For each stick
	for (int i = 0; i < 2; i++) {
		//Fill in background
		cairo_set_source_rgba(cr, stickAreaColor.r, stickAreaColor.g, stickAreaColor.b, stickAreaColor.a);
		cairo_rectangle(cr, -stickSurroundRadius, -stickSurroundRadius, stickSurroundRadius * 2, stickSurroundRadius * 2);
		cairo_fill(cr);

		//Draw crosshair
		cairo_set_line_width(cr, 1);
		cairo_set_source_rgba(cr, crosshairColor.r, crosshairColor.g, crosshairColor.b, crosshairColor.a);
		cairo_move_to(cr, -stickSurroundRadius, 0);
		cairo_line_to(cr, stickSurroundRadius, 0);
		cairo_move_to(cr, 0, -stickSurroundRadius);
		cairo_line_to(cr, 0, stickSurroundRadius);
		cairo_stroke(cr);

		//Draw circle to represent stick position
		cairo_set_source_rgba(cr, stickColor.r, stickColor.g, stickColor.b, stickColor.a);
		cairo_arc(cr, stickPositions[i * 2 + 0], stickPositions[i * 2 + 1], stickSurroundRadius / 5, 0, 2 * M_PI);
		cairo_fill(cr);

		cairo_set_source_rgba(cr, 1,1,1, 1);
		cairo_set_font_size(cr, 32);

		//Draw horizontal stick label
		int32_t labelValue;

		//Invert yaw so moving to the left goes into negative values
		if (i == 0)
			labelValue = -frame[idents.rcCommandFields[2]];
		else
			labelValue = frame[idents.rcCommandFields[(1 - i) * 2 + 0]];

		snprintf(stickLabel, sizeof(stickLabel), "%d", labelValue);
		cairo_text_extents(cr, stickLabel, &extent);

		cairo_move_to(cr, -extent.width / 2, stickSurroundRadius + extent.height + 8);
		cairo_show_text(cr, stickLabel);

		//Draw vertical stick label
		snprintf(stickLabel, sizeof(stickLabel), "%d", frame[idents.rcCommandFields[(1 - i) * 2 + 1]]);
		cairo_text_extents(cr, stickLabel, &extent);

		cairo_move_to(cr, -stickSurroundRadius - extent.width - 8, extent.height / 2);
		cairo_show_text(cr, stickLabel);

		//Advance to next stick
		cairo_translate(cr, stickSpacing, 0);
	}

	cairo_restore(cr);
}

/*
 * Draw a vertically-oriented propeller at the origin with the current source color
 */
void drawPropeller(cairo_t *cr, craft_parameters_t *parameters)
{
	cairo_move_to(cr, 0, 0);

	for (int i = 0; i < parameters->numBlades; i++) {
		cairo_curve_to(cr, parameters->tipBezierWidth, -parameters->tipBezierHeight, parameters->tipBezierWidth, parameters->bladeLength + parameters->tipBezierHeight, 0, parameters->bladeLength);
		cairo_curve_to(cr, -parameters->tipBezierWidth, parameters->bladeLength + parameters->tipBezierHeight, -parameters->tipBezierWidth, -parameters->tipBezierHeight, 0, 0);

		cairo_rotate(cr, (M_PI * 2) / parameters->numBlades);
	}

	cairo_fill(cr);
}

/**
 * Draw a craft with spinning blades at the origin
 */
void drawCraft(cairo_t *cr, int32_t *frame, double timeElapsedMicros, craft_parameters_t *parameters)
{
	static double propAngles[MAX_MOTORS] = {0};

	double angularSpeed[MAX_MOTORS];
	double rotationThisFrame[MAX_MOTORS];
	int onionLayers[MAX_MOTORS];
	int motorIndex, onion;
	double opacity;

	char motorLabel[16];
	cairo_text_extents_t extent;

	//Draw arms
	cairo_set_line_width(cr, parameters->bladeLength * 0.30);
	cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
	cairo_set_source_rgba(cr, craftColor.r, craftColor.g, craftColor.b, craftColor.a);

	for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
		cairo_move_to(cr, 0, 0);

		cairo_line_to(
			cr,
			parameters->motorSpacing * parameters->motorX[motorIndex] * 1.2,
			parameters->motorSpacing * parameters->motorY[motorIndex] * 1.2
		);
	}

	cairo_stroke(cr);

	//Draw the central hub
	cairo_move_to(cr, 0, 0);
	cairo_arc(cr, 0, 0,	parameters->motorSpacing * 0.4, 0, 2 * M_PI);
	cairo_fill(cr);

	//Compute prop speed and position
	for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
		if (idents.motorFields[motorIndex] > -1) {
			double scaled = doubleMax(frame[idents.motorFields[motorIndex]] - MINTHROTTLE, 0) / (MAXTHROTTLE - MINTHROTTLE);

			//If motors are armed (above MINTHROTTLE), keep them spinning at least a bit
			if (scaled > 0)
				scaled = scaled * 0.9 + 0.1;

			angularSpeed[motorIndex] = scaled * M_PI * 2 * MOTOR_MAX_RPS;

			rotationThisFrame[motorIndex] = angularSpeed[motorIndex] * timeElapsedMicros / 1000000;

			// Don't need to draw as many onion layers if we aren't rotating very far
			onionLayers[motorIndex] = (int) (doubleAbs(rotationThisFrame[motorIndex]) * 10);
			if (onionLayers[motorIndex] < 1)
				onionLayers[motorIndex] = 1;
		}
	}

	cairo_set_font_size(cr, 32);

	for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
		cairo_save(cr);
		{
			//Move to the motor centre
			cairo_translate(
				cr,
				parameters->motorSpacing * parameters->motorX[motorIndex],
				parameters->motorSpacing * parameters->motorY[motorIndex]
			);

			if (options.propStyle == PROP_STYLE_BLADES) {
				//Draw several copies of the prop along its movement path so we can simulate motion blur
				for (onion = 1; onion <= onionLayers[motorIndex]; onion++) {
					cairo_save(cr);
					{
						// Opacity falls when the motor is spinning closer to max speed
						opacity = 1.0 / (onionLayers[motorIndex] / 2.0);

						cairo_set_source_rgba(
							cr,
							parameters->propColor[motorIndex].r,
							parameters->propColor[motorIndex].g,
							parameters->propColor[motorIndex].b,
							/* Fade in the blade toward its rotational direction, but don't fade to zero */
							opacity * ((((double) onion / onionLayers[motorIndex]) + 1.0) / 2)
						);

						cairo_rotate(cr, (propAngles[motorIndex] + (rotationThisFrame[motorIndex] * onion) / onionLayers[motorIndex]) * parameters->motorDirection[motorIndex]);

						drawPropeller(cr, parameters);
					}
					cairo_restore(cr);
				}
			} else {
				cairo_set_source_rgba(
					cr,
					parameters->propColor[motorIndex].r / 2,
					parameters->propColor[motorIndex].g / 2,
					parameters->propColor[motorIndex].b / 2,
					0.5
				);

				cairo_move_to(cr, 0, 0);
				cairo_arc(cr, 0, 0, parameters->bladeLength, 0, M_PI * 2);
				cairo_fill(cr);

				cairo_set_source_rgba(
					cr,
					parameters->propColor[motorIndex].r,
					parameters->propColor[motorIndex].g,
					parameters->propColor[motorIndex].b,
					1
				);

				cairo_move_to(cr, 0, 0);
				cairo_arc(cr, 0, 0, parameters->bladeLength, -M_PI_2, -M_PI_2 + M_PI * 2 * doubleMax(frame[idents.motorFields[motorIndex]] - MINTHROTTLE, 0) / (MAXTHROTTLE - MINTHROTTLE));
				cairo_fill(cr);
			}

			snprintf(motorLabel, sizeof(motorLabel), "%d", frame[idents.motorFields[motorIndex]]);

			cairo_text_extents(cr, motorLabel, &extent);

			if (parameters->motorX[motorIndex] > 0)
				cairo_translate(cr, parameters->bladeLength + 10, 0);
			else
				cairo_translate(cr, -(parameters->bladeLength + 10 + extent.width), 0);

			cairo_move_to(cr, 0, 0);

			cairo_set_source_rgb(
				cr,
				doubleMin(parameters->propColor[motorIndex].r * 1.25, 1),
				doubleMin(parameters->propColor[motorIndex].g * 1.25, 1),
				doubleMin(parameters->propColor[motorIndex].b * 1.25, 1)
			);

			cairo_show_text (cr, motorLabel);
		}
		cairo_restore(cr);
	}

	for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++)
		propAngles[motorIndex] += rotationThisFrame[motorIndex];
}

void decideCraftParameters(craft_parameters_t *parameters, int imageWidth, int imageHeight)
{
	(void) imageHeight;

	parameters->numMotors = 4;
	parameters->numBlades = 2;
	parameters->bladeLength = imageWidth / 25;
	parameters->tipBezierWidth = 0.2 * parameters->bladeLength;
	parameters->tipBezierHeight = 0.1 * parameters->bladeLength;
	parameters->motorSpacing = parameters->bladeLength * 1.15;

	parameters->motorX[0] = 1;
	parameters->motorY[0] = 1;
	parameters->motorX[1] = 1;
	parameters->motorY[1] = -1;
	parameters->motorX[2] = -1;
	parameters->motorY[2] = 1;
	parameters->motorX[3] = -1;
	parameters->motorY[3] = -1;

	parameters->motorDirection[0] = 1;
	parameters->motorDirection[1] = -1;
	parameters->motorDirection[2] = -1;
	parameters->motorDirection[3] = 1;

	//TODO we can let the user choose their prop colours to match their model if they like
	for (int i = 0; i < parameters->numMotors; i++)
		parameters->propColor[i] = idents.motorColors[i];
}

/**
 * Plot the given field within the specified time period. valueYOffset will be added from the values before
 * plotting. When the value reaches valueYRange it'll be drawn plotHeight pixels away from the origin.
 */
void plotLine(cairo_t *cr, color_t color, uint32_t windowStartTime, uint32_t windowEndTime, int firstFrameIndex,
		int fieldIndex, int valueYOffset, int valueYRange, int plotHeight, bool stretchedScale) {

	uint32_t windowWidthMicros = windowEndTime - windowStartTime;
	int32_t fieldValue;
	int64_t frameTime;

	bool drawingLine = false;

	//Draw points from this line until we leave the window
	for (int frameIndex = firstFrameIndex; frameIndex < points->frameCount; frameIndex++) {
		if (datapointsGetFieldAtIndex(points, frameIndex, fieldIndex, &fieldValue)) {
			datapointsGetTimeAtIndex(points, frameIndex, &frameTime);

			fieldValue += valueYOffset;

			double nextX, nextY;

			if (stretchedScale) {
				//Exaggerate the values close to zero (used for PIDs which spend most of their time there)
				uint32_t fieldValueAbs = fieldValue < 0 ? -fieldValue : fieldValue;

				nextY = 0;

				nextY += minS32(fieldValueAbs, 25) * 3;
				nextY += maxS32(fieldValueAbs - 25, 0) / 2.0;
				nextY -= maxS32(fieldValueAbs - 100, 0) / 4.0;

				nextY *= fieldValue < 0 ? -1 : 1;
			} else {
				nextY = fieldValue;
			}

			nextY = (double) -nextY / valueYRange * plotHeight;
			nextX = (double)(frameTime - windowStartTime) / windowWidthMicros * options.imageWidth;

			if (drawingLine) {
				cairo_line_to(cr, nextX, nextY);
			} else {
				cairo_move_to(cr, nextX, nextY);
				drawingLine = true;
			}

			if (frameTime >= windowEndTime)
				break;
		} else {
			//We'll need to start a new line at the next point
			drawingLine = false;
		}
	}

	cairo_set_source_rgb(cr, color.r, color.g, color.b);
	cairo_stroke(cr);
}

void drawPIDTable(cairo_t *cr, int32_t *frame)
{
	cairo_font_extents_t fontExtent;

	cairo_font_extents(cr, &fontExtent);

	const int INTERROW_SPACING = 32;
	const int VERT_SPACING = fontExtent.height + INTERROW_SPACING;
	const int FIRST_ROW_TOP = fontExtent.height + INTERROW_SPACING;
	const int HORZ_SPACING = 110, FIRST_COL_LEFT = 140;

	const int HORZ_EXTENT = FIRST_COL_LEFT + HORZ_SPACING * 5 - 30;
	const int VERT_EXTENT = FIRST_ROW_TOP + fontExtent.height * 3 + INTERROW_SPACING * 2;

	const int PADDING = 32;

	char fieldLabel[16];
	int pidType, axisIndex;
	const char *pidName;

	cairo_save(cr);

	//Centre about the origin
	cairo_translate(cr, -HORZ_EXTENT / 2, -VERT_EXTENT / 2);

	//Draw a background box
	cairo_set_source_rgba(cr, 0, 0, 0, 0.33);

	cairo_rectangle(cr, -PADDING, -PADDING, HORZ_EXTENT + PADDING * 2, VERT_EXTENT + PADDING * 2);

	cairo_fill(cr);

	cairo_set_font_size(cr, 30);
	cairo_set_source_rgb(cr, 1, 1, 1);

	//Draw field labels first
	for (pidType = PID_P - 1; pidType <= PID_TOTAL; pidType++) {
		switch (pidType) {
			case PID_P - 1:
				pidName = "Gyro";
			break;
			case PID_P:
				pidName = "P";
			break;
			case PID_I:
				pidName = "I";
			break;
			case PID_D:
				pidName = "D";
			break;
			case PID_TOTAL:
				pidName = "Sum";
			break;
		}
		cairo_move_to (cr, (pidType + 1) * HORZ_SPACING + FIRST_COL_LEFT, fontExtent.height);
		cairo_show_text (cr, pidName);
	}

	for (axisIndex = 0; axisIndex < 3; axisIndex++) {
		switch (axisIndex) {
			case 0:
				pidName = "Roll";
			break;
			case 1:
				pidName = "Pitch";
			break;
			case 2:
				pidName = "Yaw";
			break;
		}

		cairo_set_source_rgb(
			cr,
			idents.PIDAxisColors[PID_I][axisIndex].r,
			idents.PIDAxisColors[PID_I][axisIndex].g,
			idents.PIDAxisColors[PID_I][axisIndex].b
		);

		cairo_move_to (cr, 0, FIRST_ROW_TOP + axisIndex * VERT_SPACING + fontExtent.height);
		cairo_show_text (cr, pidName);
	}

	//Now draw the values
	for (pidType = PID_P - 1; pidType <= PID_TOTAL; pidType++) {
		for (axisIndex = 0; axisIndex < 3; axisIndex++) {
			int32_t fieldValue;

			if (pidType == PID_P - 1) {
				if (idents.hasGyros)
					fieldValue = frame[idents.gyroFields[axisIndex]];
				else
					fieldValue = 0;
			} else if (idents.hasPIDs) {
				if (pidType == PID_TOTAL)
					fieldValue = frame[idents.axisPIDFields[PID_P][axisIndex]] + frame[idents.axisPIDFields[PID_I][axisIndex]] - frame[idents.axisPIDFields[PID_D][axisIndex]];
				else
					fieldValue = frame[idents.axisPIDFields[pidType][axisIndex]];
			} else
				fieldValue = 0;

			snprintf(fieldLabel, sizeof(fieldLabel), "%d", fieldValue);

			switch (pidType) {
				case PID_P - 1:
					cairo_set_source_rgb(
						cr,
						idents.gyroColors[axisIndex].r,
						idents.gyroColors[axisIndex].g,
						idents.gyroColors[axisIndex].b
					);
				break;
				case PID_TOTAL:
					cairo_set_source_rgb(
						cr,
						idents.PIDAxisColors[PID_D][axisIndex].r,
						idents.PIDAxisColors[PID_D][axisIndex].g,
						idents.PIDAxisColors[PID_D][axisIndex].b
					);
				break;
				default:
					cairo_set_source_rgb(
						cr,
						idents.PIDAxisColors[pidType][axisIndex].r,
						idents.PIDAxisColors[pidType][axisIndex].g,
						idents.PIDAxisColors[pidType][axisIndex].b
					);
				break;
			}

			cairo_move_to (
				cr,
				FIRST_COL_LEFT + (pidType + 1) * HORZ_SPACING,
				FIRST_ROW_TOP + axisIndex * VERT_SPACING + fontExtent.height
			);
			cairo_show_text (cr, fieldLabel);
		}
	}

	cairo_restore(cr);
}

void drawCenterline(cairo_t *cr)
{
	cairo_save(cr);

	//Draw an origin line
	cairo_set_source_rgba(cr, 1, 1, 1, 0.5);

	cairo_set_dash(cr, 0, 0, 0);
	cairo_set_line_width(cr, 1);
	cairo_move_to(cr, 0, 0);
	cairo_line_to(cr, options.imageWidth, 0);
	cairo_stroke(cr);

	cairo_restore(cr);
}

void drawAxisLabel(cairo_t *cr, const char *axisLabel)
{
	cairo_text_extents_t extent;

	cairo_set_font_size(cr, 28);
	cairo_set_source_rgba(cr, 1, 1, 1, 0.9);

	cairo_text_extents(cr, axisLabel, &extent);
	cairo_move_to(cr, options.imageWidth - 8 - extent.width, -8);
	cairo_show_text(cr, axisLabel);
}

void* pngRenderThread(void *arg)
{
	char filename[255];
	pngRenderingTask_t *task = (pngRenderingTask_t *) arg;

    snprintf(filename, sizeof(filename), "%s%06d.png", options.outputPrefix, task->outputFrameIndex);
    cairo_surface_write_to_png (task->surface, filename);
    cairo_surface_destroy (task->surface);

    //Release our slot in the rendering pool, we're done
#ifdef __APPLE__
    dispatch_semaphore_signal(pngRenderingSem);
#else
    sem_post(&pngRenderingSem);
#endif

    return 0;
}

void drawFrameLabel(cairo_t *cr, uint32_t frameIndex, int32_t frameTime)
{
	char frameNumberBuf[16];
	cairo_text_extents_t extentFrameNumber, extentFrameTime;

	snprintf(frameNumberBuf, sizeof(frameNumberBuf), "#%07u", frameIndex);

	cairo_set_font_size(cr, 24);
	cairo_set_source_rgba(cr, 1, 1, 1, 0.65);

	cairo_text_extents(cr, "#0000000", &extentFrameNumber);

	cairo_move_to(cr, options.imageWidth - extentFrameNumber.width - 8, options.imageHeight - 8);
	cairo_show_text(cr, frameNumberBuf);

	int frameMsec, frameSec, frameMins;

	frameMsec = frameTime / 1000;

	frameSec = frameMsec / 1000;
	frameMsec %= 1000;

	frameMins = frameSec / 60;
	frameSec %= 60;

	snprintf(frameNumberBuf, sizeof(frameNumberBuf), "%02d:%02d.%03d", frameMins, frameSec, frameMsec);

	cairo_text_extents(cr, "00:00.000", &extentFrameTime);

	cairo_move_to(cr, options.imageWidth - extentFrameTime.width - 8, options.imageHeight - 8 - extentFrameNumber.height - 8);
	cairo_show_text(cr, frameNumberBuf);
}

void renderPoints(int64_t startTime, uint64_t endTime)
{
	//Change how much data is displayed at one time
	const int windowWidthMicros = 1000 * 1000;

	const int PID_SCALE = 200;
	const int GYRO_SCALE = PID_SCALE;

	//Bring the current time into the centre of the plot
	const int startXTimeOffset = windowWidthMicros / 2;

	int i;

	uint64_t lastCenterTime;
	int64_t frameTime;
	uint32_t totalDurationMicro = endTime - startTime;
	int32_t outputFrames = (int32_t) (((int64_t) totalDurationMicro * options.fps) / 1000000 + 1);
	int32_t frameValues[FLIGHT_LOG_MAX_FIELDS];

	struct craftDrawingParameters_t craftParameters;

#ifdef __APPLE__
	pngRenderingSem = dispatch_semaphore_create(PNG_RENDERING_THREADS);
#else
	sem_init(&pngRenderingSem, 0, PNG_RENDERING_THREADS);
#endif

	decideCraftParameters(&craftParameters, options.imageWidth, options.imageHeight);

	int durationSecs = (endTime - startTime) / 1000000;
	int durationMins = durationSecs / 60;
	durationSecs %= 60;

	fprintf(stderr, "%d frames to be rendered at %d FPS [%d:%2d]\n", outputFrames, options.fps, durationMins, durationSecs);
	fprintf(stderr, "\n");

	for (int outputFrameIndex = 0; outputFrameIndex < outputFrames; outputFrameIndex++) {
		int64_t windowCenterTime = (int32_t) (startTime + ((int64_t) outputFrameIndex * 1000000) / options.fps);
		int64_t windowStartTime = windowCenterTime - startXTimeOffset;
		int64_t windowEndTime = windowStartTime + windowWidthMicros;

	    cairo_surface_t *surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, options.imageWidth, options.imageHeight);
	    cairo_t *cr = cairo_create (surface);

		// Find the frame just to the left of the first pixel so we can start drawing lines from there
		int firstFrameIndex = datapointsFindFrameAtTime(points, windowStartTime - 1);

		if (firstFrameIndex == -1) {
			firstFrameIndex = 0;
		}

		cairo_select_font_face (cr, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

	    //Plot the upper motor graph
	    if (options.plotMotors) {
			cairo_save(cr);
			{
				if (options.plotPids) {
					cairo_translate(cr, 0, options.imageHeight * 0.15);
				} else {
					cairo_translate(cr, 0, options.imageHeight * 0.25);
				}

				drawCenterline(cr);

				cairo_set_line_width(cr, 2.5);

				for (i = 0; i < idents.numMotors; i++) {
					plotLine(cr, idents.motorColors[i], windowStartTime,
							windowEndTime, firstFrameIndex, idents.motorFields[i], -(MINTHROTTLE + MAXTHROTTLE) / 2,
							(MAXTHROTTLE - MINTHROTTLE) / 2, options.imageHeight * (options.plotPids ? 0.15 : 0.20),
							false);
				}

				drawAxisLabel(cr, "Motors");
			}
			cairo_restore(cr);
		}

	    //Plot the lower PID graphs
	    cairo_save(cr);
	    {
	    	if (options.plotPids) {
	    		//Plot three axes as different graphs
		    	cairo_translate(cr, 0, options.imageHeight * 0.60);
				for (int axis = 0; axis < 3; axis++) {
					cairo_save(cr);

					cairo_translate(cr, 0, options.imageHeight * 0.2 * (axis - 1));

					drawCenterline(cr);

					for (int pidType = PID_D; pidType >= PID_P; pidType--) {
						switch (pidType) {
							case PID_P:
								cairo_set_line_width(cr, 2.8);
							break;
							case PID_I:
								cairo_set_dash(cr, DASHED_LINE, DASHED_LINE_NUM_POINTS, 0);
								cairo_set_line_width(cr, 2);
							break;
							case PID_D:
								cairo_set_line_width(cr, 2);
						}

						plotLine(cr, idents.PIDAxisColors[pidType][axis], windowStartTime,
								windowEndTime, firstFrameIndex, idents.axisPIDFields[pidType][axis], 0, PID_SCALE, options.imageHeight * 0.15,
								true);

						cairo_set_dash(cr, 0, 0, 0);
					}

					if (options.plotGyros) {
						cairo_set_dash(cr, DOTTED_LINE, DOTTED_LINE_NUM_POINTS, 0);
						cairo_set_line_width(cr, 2);

						plotLine(cr, idents.gyroColors[axis], windowStartTime,
								windowEndTime, firstFrameIndex, idents.gyroFields[axis], 0, GYRO_SCALE, options.imageHeight * 0.15,
								true);

						cairo_set_dash(cr, 0, 0, 0);
					}

					const char *axisLabel;
					if (options.plotGyros) {
						switch (axis) {
							case 0:
								axisLabel = "Gyro + PIDs roll";
							break;
							case 1:
								axisLabel = "Gyro + PIDs pitch";
							break;
							case 2:
								axisLabel = "Gyro + PIDs yaw";
							break;
						}
					} else {
						switch (axis) {
							case 0:
								axisLabel = "Roll PIDs";
							break;
							case 1:
								axisLabel = "Pitch PIDs";
							break;
							case 2:
								axisLabel = "Yaw PIDs";
							break;
						}
					}

					drawAxisLabel(cr, axisLabel);

					cairo_restore(cr);
				}
			} else if (options.plotGyros) {
				//Plot three gyro axes on one graph
		    	cairo_translate(cr, 0, options.imageHeight * 0.70);

				drawCenterline(cr);

				for (int axis = 0; axis < 3; axis++) {
					plotLine(cr, idents.gyroColors[axis], windowStartTime,
							windowEndTime, firstFrameIndex, idents.gyroFields[axis], 0, GYRO_SCALE, options.imageHeight * 0.25,
							true);
				}

				drawAxisLabel(cr, "Gyro");
			}

			//Plot other misc fields
			for (i = 0; i < idents.numMisc; i++) {
				plotLine(cr, idents.miscColors[i], windowStartTime,
						windowEndTime, firstFrameIndex, idents.miscFields[i], 0, 400, options.imageHeight / 4,
						false);
			}
		}
		cairo_restore(cr);

		//Draw a bar highlighting the current time
		cairo_set_source_rgba(cr, 1, 0.25, 0.25, 0.2);
		cairo_set_line_width(cr, 20);

		double centerX = options.imageWidth / 2.0;

		cairo_move_to(cr, centerX, 0);
		cairo_line_to(cr, centerX, options.imageHeight);
		cairo_stroke(cr);

		int centerFrameIndex = datapointsFindFrameAtTime(points, windowCenterTime);

		//Draw the command stick positions from the centered frame
		if (datapointsGetFrameAtIndex(points, centerFrameIndex, &frameTime, frameValues)) {
			if (options.drawSticks) {
				cairo_save(cr);
				{
					cairo_translate(cr, 0.75 * options.imageWidth, 0.20 * options.imageHeight);

					drawCommandSticks(frameValues, options.imageWidth, options.imageHeight, cr);
				}
				cairo_restore(cr);
			}

			if (options.drawPidTable) {
				cairo_save(cr);
				{
					cairo_translate(cr, 0.25 * options.imageWidth, 0.75 * options.imageHeight);
					drawPIDTable(cr, frameValues);
				}
				cairo_restore(cr);
			}

			if (options.drawCraft) {
				cairo_save(cr);
				{
					cairo_translate(cr, 0.25 * options.imageWidth, 0.20 * options.imageHeight);
					drawCraft(cr, frameValues, outputFrameIndex > 0 ? windowCenterTime - lastCenterTime : 0, &craftParameters);
				}
				cairo_restore(cr);
			}
		}

		drawFrameLabel(cr, centerFrameIndex > -1 ? centerFrameIndex : 0, windowCenterTime - flightLog->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]);

	    cairo_destroy(cr);

		lastCenterTime = windowCenterTime;

		//Encode the PNG using the thread pool, since it's really expensive
	    pthread_t thread;
	    pngRenderingTask_t *task = (pngRenderingTask_t*) malloc(sizeof(*task));

	    task->surface = surface;
	    task->outputFrameIndex = outputFrameIndex;

	    // Reserve a slot in the rendering pool...
#ifdef __APPLE__
	    dispatch_semaphore_wait(pngRenderingSem, DISPATCH_TIME_FOREVER);
#else
	    sem_wait(&pngRenderingSem);
#endif

	    pthread_create(&thread, NULL, pngRenderThread, task);

	    if ((outputFrameIndex + 1) % 500 == 0) {
			fprintf(stderr, "Rendered %d frames (%.1f%%)...\n", outputFrameIndex + 1, (double)(outputFrameIndex + 1) / outputFrames * 100);
	    }
	}

	// Wait for all the PNG rendering to complete
	for (i = 0; i < PNG_RENDERING_THREADS; i++) {
#ifdef __APPLE__
		dispatch_semaphore_wait(pngRenderingSem, DISPATCH_TIME_FOREVER);
#else
		sem_wait(&pngRenderingSem);
#endif
	}
}

void printUsage(const char *argv0)
{
	fprintf(stderr,
		"Blackbox flight log renderer by Nicholas Sherlock\n\n"
		"Usage:\n"
		"     %s [options] <logfilename.txt>\n\n"
		"Options:\n"
		"   --help                 This page\n"
		"   --index <num>          Choose the log from the file that should be rendered\n"
		"   --width <px>           Choose the width of the image (default %d)\n"
		"   --height <px>          Choose the height of the image (default %d)\n"
		"   --fps                  FPS of the resulting video (default %d)\n"
		"   --start <x:xx>         Begin the log at this time offset (default 0:00)\n"
		"   --end <x:xx>           End the log at this time offset\n"
		"   --[no-]draw-pid-table  Show table with PIDs and gyros (default on)\n"
		"   --[no-]draw-craft      Show craft drawing (default on)\n"
		"   --[no-]draw-sticks     Show RC command sticks (default on)\n"
		"   --[no-]plot-motor      Draw motors on the upper graph (default on)\n"
		"   --[no-]plot-pid        Draw PIDs on the lower graph (default off)\n"
		"   --[no-]plot-gyro       Draw gyroscopes on the lower graph (default on)\n"
		"   --smoothing-pid <n>    Smoothing window for the PIDs (default %d)\n"
		"   --smoothing-gyro <n>   Smoothing window for the gyroscopes (default %d)\n"
		"   --smoothing-motor <n>  Smoothing window for the motors (default %d)\n"
		"   --prop-style           Style of propeller display (pie/blades, default %s)\n"
		"\n", argv0, defaultOptions.imageWidth, defaultOptions.imageHeight, defaultOptions.fps, defaultOptions.pidSmoothing,
			defaultOptions.gyroSmoothing, defaultOptions.motorSmoothing, defaultOptions.propStyle == PROP_STYLE_BLADES ? "blades" : "pie"
	);
}

void parseCommandlineOptions(int argc, char **argv)
{
	int option_index = 0;
	int c;

	memcpy(&options, &defaultOptions, sizeof(options));

	while (1)
	{
		static struct option long_options[] = {
			{"index", required_argument, 0, 'i'},
			{"width", required_argument, 0, 'w'},
			{"height", required_argument, 0, 'h'},
			{"fps", required_argument, 0, 'f'},
			{"start", required_argument, 0, 'b'},
			{"end", required_argument, 0, 'e'},
			{"plot-pid", no_argument, 0, 'p'},
			{"plot-gyro", no_argument, 0, 'g'},
			{"plot-motor", no_argument, 0, 'm'},
			{"no-plot-pid", no_argument, 0, 'P'},
			{"no-plot-gyro", no_argument, 0, 'G'},
			{"no-plot-motor", no_argument, 0, 'M'},
			{"draw-pid-table", no_argument, 0, 't'},
			{"draw-craft", no_argument, 0, 'c'},
			{"draw-sticks", no_argument, 0, 's'},
			{"no-draw-pid-table", no_argument, 0, 'T'},
			{"no-draw-craft", no_argument, 0, 'C'},
			{"no-draw-sticks", no_argument, 0, 'S'},
			{"smoothing-pid", required_argument, 0, '1'},
			{"smoothing-gyro", required_argument, 0, '2'},
			{"smoothing-motor", required_argument, 0, '3'},
			{"prop-style", required_argument, 0, 'r'},
			{0, 0, 0, 0}
		};

		opterr = 0;

		c = getopt_long (argc, argv, ":", long_options, &option_index);

		if (c == -1)
			break;

		switch (c) {
			case 'b':
				options.timeStart = atoi(optarg);
			break;
			case 'e':
				options.timeEnd = atoi(optarg);
			break;
			case 'w':
				options.imageWidth = atoi(optarg);
			break;
			case 'h':
				options.imageHeight = atoi(optarg);
			break;
			case 'f':
				options.fps = atoi(optarg);
			break;
			case 'p':
				options.plotPids = true;
			break;
			case 'g':
				options.plotGyros = true;
			break;
			case 'm':
				options.plotMotors = true;
			break;
			case 'P':
				options.plotPids = false;
			break;
			case 'G':
				options.plotGyros = false;
			break;
			case 'M':
				options.plotMotors = false;
			break;
			case 't':
				options.drawPidTable = true;
			break;
			case 'c':
				options.drawCraft = true;
			break;
			case 's':
				options.drawSticks = true;
			break;
			case 'T':
				options.drawPidTable = false;
			break;
			case 'C':
				options.drawCraft = false;
			break;
			case 'S':
				options.drawSticks = false;
			break;
			case '1':
				options.pidSmoothing = atoi(optarg);
			break;
			case '2':
				options.gyroSmoothing = atoi(optarg);
			break;
			case '3':
				options.motorSmoothing = atoi(optarg);
			break;
			case 'i':
				options.logNumber = atoi(optarg);
			break;
			case 'r':
				if (strcmp(optarg, "pie") == 0) {
					options.propStyle = PROP_STYLE_PIE_CHART;
				} else {
					options.propStyle = PROP_STYLE_BLADES;
				}
			break;
		    case ':':
		        /* missing option argument */
		        fprintf(stderr, "%s: option '-%c' requires an argument\n", argv[0], optopt);
		        exit(-1);
			break;
		    case '?':
		    default:
		        /* invalid option */
		        fprintf(stderr, "%s: option '-%c' is invalid: ignored\n", argv[0], optopt);
		        exit(-1);
			break;
		}
	}

	if (optind < argc) {
		options.filename = argv[optind];

		char *extension = strrchr(options.filename, '.');
		int sourceLen, prefixLen;

		if (extension) {
			sourceLen = extension - options.filename;
		} else {
			sourceLen = strlen(options.filename);
		}

		prefixLen = sourceLen + 1;

		options.outputPrefix = malloc(prefixLen * sizeof(*options.outputPrefix));

		for (int i = 0; i < sourceLen; i++)
			options.outputPrefix[i] = options.filename[i];

		options.outputPrefix[sourceLen] = '.';
		options.outputPrefix[prefixLen] = '\0';
	}
}

static void applySmoothing() {
	if (options.gyroSmoothing && idents.hasGyros) {
		for (int axis = 0; axis < 3; axis++)
			datapointsSmoothField(points, idents.gyroFields[axis], options.gyroSmoothing);
	}

	if (options.pidSmoothing && idents.hasPIDs) {
		for (int pid = PID_P; pid <= PID_D; pid++)
			for (int axis = 0; axis < 3; axis++)
				datapointsSmoothField(points, idents.axisPIDFields[pid][axis], options.pidSmoothing);
	}

	if (options.motorSmoothing) {
		for (int motor = 0; motor < idents.numMotors; motor++)
			datapointsSmoothField(points, idents.motorFields[motor], options.motorSmoothing);
	}
}

int chooseLog(flightLog_t *log)
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

int main(int argc, char **argv)
{
	uint32_t timeStart, timeEnd;
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

    flightLog = flightLogCreate(fd);

    logIndex = chooseLog(flightLog);

    if (logIndex == -1)
    	return -1;

	//First check out how many frames we need to store so we can pre-allocate (parsing will update the flightlog stats which contain that info)
	flightLogParse(flightLog, logIndex, 0, 0, false);

	// Don't include the leading time or field index fields in the field names / counts
	points = datapointsCreate(flightLog->fieldCount - 2, flightLog->fieldNames + 2, flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1);

	//Now decode that data into the points array
	flightLogParse(flightLog, logIndex, 0, loadFrameIntoPoints, false);

	identifyFields();

	applySmoothing();

	timeStart = flightLog->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME] + options.timeStart * 1000000u;

	if (options.timeEnd == 0)
		timeEnd = flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME];
	else {
		timeEnd = flightLog->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME] + options.timeEnd * 1000000u;

		if (timeEnd > flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME])
			timeEnd = flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME];
	}

	if (timeEnd < timeStart) {
		fprintf(stderr, "Selected time range excludes all datapoints\n");
		return -1;
	}

	renderPoints(timeStart, timeEnd);

    return 0;
}
