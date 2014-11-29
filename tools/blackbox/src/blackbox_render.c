#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <getopt.h>
#include <errno.h>

#include <fcntl.h>

#include <cairo.h>

#include <ft2build.h>
#include FT_FREETYPE_H
#include "embeddedfont.h"

#include "platform.h"
#include "tools.h"
#include "parser.h"
#include "datapoints.h"
#include "expo.h"
#include "imu.h"

#define MAX_MOTORS 8

//Controls how fast the props spin on the video
#define MOTOR_MAX_RPS 25

#define FONTSIZE_CURRENT_VALUE_LABEL 36
#define FONTSIZE_PID_TABLE_LABEL 34
#define FONTSIZE_AXIS_LABEL 34
#define FONTSIZE_FRAME_LABEL 32

#define PNG_RENDERING_THREADS 3

#define DATAPOINTS_EXTRA_COMPUTED_FIELDS 6

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
	int outputLogIndex, outputFrameIndex;
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

	int plotPids, plotPidSum, plotGyros, plotMotors;
	int drawPidTable, drawSticks, drawCraft;

	int pidSmoothing, gyroSmoothing, motorSmoothing;

	PropStyle propStyle;

	//Start and end time of video in seconds offset from the beginning of the log
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

	bool hasAccs;
	int accFields[3];
	color_t accColors[3];

	int numMisc;
	int miscFields[FLIGHT_LOG_MAX_FIELDS];
	color_t miscColors[FLIGHT_LOG_MAX_FIELDS];

	int roll, pitch, heading;
	int axisPIDSum[3];
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

static const colorAlpha_t stickColor = {1, 0.4, 0.4, 1.0};
static const colorAlpha_t stickAreaColor = {0.3, 0.3, 0.3, 0.8};
static const colorAlpha_t craftColor = {0.3, 0.3, 0.3, 1};
static const colorAlpha_t crosshairColor = {0.75, 0.75, 0.75, 0.5};

static const renderOptions_t defaultOptions = {
	.imageWidth = 1920, .imageHeight = 1080,
	.fps = 30, .help = 0, .propStyle = PROP_STYLE_PIE_CHART,
	.plotPids = false, .plotPidSum = false, .plotGyros = true, .plotMotors = true,
	.pidSmoothing = 4, .gyroSmoothing = 2, .motorSmoothing = 2,
	.drawCraft = true, .drawPidTable = true, .drawSticks = true,
	.filename = 0,
	.timeStart = 0, .timeEnd = 0,
	.logNumber = 0
};

//Cairo doesn't include this in any header (apparently it is considered private?)
extern cairo_font_face_t* cairo_ft_font_face_create_for_ft_face(FT_Face face, int load_flags);

static renderOptions_t options;
static expoCurve_t *pitchStickCurve, *pidCurve, *gyroCurve, *accCurve, *motorCurve;

static semaphore_t pngRenderingSem;
static bool pngRenderingSemCreated = false;

static flightLog_t *flightLog;
static Datapoints *points;
static int selectedLogIndex;

//Information about fields we have classified
static fieldIdentifications_t idents;

static FT_Library freetypeLibrary;

void loadFrameIntoPoints(flightLog_t *log, bool frameValid, int32_t *frame, int frameOffset, int frameSize)
{
	(void) log;
	(void) frameSize;
	(void) frameOffset;

	/*
	 * Pull the first two fields (iteration and time) off the front of the frame fields,
	 * since datapoints handles those as separate arguments in this call:
	 */
	if (frameValid)
		datapointsSetFrame(points, frame[FLIGHT_LOG_FIELD_INDEX_ITERATION], frame[FLIGHT_LOG_FIELD_INDEX_TIME], frame + 2);
}

/**
 * Examine the field metadata from the flight log and assign their details to the "idents" global.
 */
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

	for (int axis = 0; axis < 3; axis++) {
		idents.axisPIDSum[axis] = -1;
		idents.accFields[axis] = -1;
	}

	idents.roll = idents.pitch = idents.heading = -1;
	idents.hasGyros = false;
	idents.hasPIDs = false;
	idents.hasAccs = false;

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
		} else if (strncmp(points->fieldNames[fieldIndex], "axisPID[", strlen("axisPID[")) == 0) {
			int axisIndex = atoi(points->fieldNames[fieldIndex] + strlen("axisPID["));

			idents.axisPIDSum[axisIndex] = fieldIndex;
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
		} else if (strncmp(points->fieldNames[fieldIndex], "gyroData[", strlen("gyroData[")) == 0) {
			int axisIndex = atoi(points->fieldNames[fieldIndex] + strlen("gyroData["));

			idents.hasGyros = true;
			idents.gyroFields[axisIndex] = fieldIndex;
			idents.gyroColors[axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];
		} else if (strncmp(points->fieldNames[fieldIndex], "accSmooth[", strlen("accSmooth[")) == 0) {
			int axisIndex = atoi(points->fieldNames[fieldIndex] + strlen("accSmooth["));

			idents.hasAccs = true;
			idents.accFields[axisIndex] = fieldIndex;
			idents.accColors[axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];
		} else if (strcmp(points->fieldNames[fieldIndex], "roll") == 0) {
			idents.roll = fieldIndex;
		} else if (strcmp(points->fieldNames[fieldIndex], "pitch") == 0) {
			idents.pitch = fieldIndex;
		} else if (strcmp(points->fieldNames[fieldIndex], "heading") == 0) {
			idents.heading = fieldIndex;
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
	const int yawStickMax = 500;
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

	//Compute the position of the sticks in the range [-1..1] (left stick x, left stick y, right stick x, right stick y)
	double stickPositions[4];

	stickPositions[0] = -rcCommand[2] / yawStickMax; //Yaw
	stickPositions[1] = (1500 - rcCommand[3]) / 500; //Throttle
	stickPositions[2] = expoCurveLookup(pitchStickCurve, rcCommand[0]); //Roll
	stickPositions[3] = expoCurveLookup(pitchStickCurve, -rcCommand[1]); //Pitch

	for (stickIndex = 0; stickIndex < 4; stickIndex++) {
		//Clamp to [-1..1]
		stickPositions[stickIndex] = stickPositions[stickIndex] > 1 ? 1 : (stickPositions[stickIndex] < -1 ? -1 : stickPositions[stickIndex]);

		//Scale to our stick size
		stickPositions[stickIndex] *= stickSurroundRadius;
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
		cairo_set_font_size(cr, FONTSIZE_CURRENT_VALUE_LABEL);

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

	/*if (idents.heading) {
		cairo_rotate(cr, intToFloat(frame[idents.heading]));
	}*/

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
			double scaled = doubleMax(frame[idents.motorFields[motorIndex]] - (int32_t) flightLog->minthrottle, 0) / (flightLog->maxthrottle - flightLog->minthrottle);

			//If motors are armed (above minthrottle), keep them spinning at least a bit
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

	cairo_set_font_size(cr, FONTSIZE_CURRENT_VALUE_LABEL);

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
				cairo_arc(cr, 0, 0, parameters->bladeLength, -M_PI_2, -M_PI_2 + M_PI * 2 * doubleMax(frame[idents.motorFields[motorIndex]] - (int32_t) flightLog->minthrottle, 0) / (flightLog->maxthrottle - flightLog->minthrottle));
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
		int fieldIndex, expoCurve_t *curve, int plotHeight)
{
	uint32_t windowWidthMicros = windowEndTime - windowStartTime;
	int32_t fieldValue;
	int64_t frameTime;

	bool drawingLine = false;

	//Draw points from this line until we leave the window
	for (int frameIndex = firstFrameIndex; frameIndex < points->frameCount; frameIndex++) {
		if (datapointsGetFieldAtIndex(points, frameIndex, fieldIndex, &fieldValue)) {
			datapointsGetTimeAtIndex(points, frameIndex, &frameTime);

			double nextX, nextY;

			nextY = (double) -expoCurveLookup(curve, fieldValue) * plotHeight;
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
	const int HORZ_SPACING = 100, FIRST_COL_LEFT = 140;

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

	cairo_set_font_size(cr, FONTSIZE_PID_TABLE_LABEL);
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
				else if (pidType == PID_D)
					/*
					 * It seems kinda confusing that D is subtracted from the PID sum, how about I just negate it so that
					 * the sum is a simple P + I + D?
					 */
					fieldValue = -frame[idents.axisPIDFields[pidType][axisIndex]];
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

	cairo_set_font_size(cr, FONTSIZE_AXIS_LABEL);
	cairo_set_source_rgba(cr, 1, 1, 1, 0.9);

	cairo_text_extents(cr, axisLabel, &extent);
	cairo_move_to(cr, options.imageWidth - 8 - extent.width, -8);
	cairo_show_text(cr, axisLabel);
}

void drawFrameLabel(cairo_t *cr, uint32_t frameIndex, int32_t frameTime)
{
	char frameNumberBuf[16];
	cairo_text_extents_t extentFrameNumber, extentFrameTime;

	snprintf(frameNumberBuf, sizeof(frameNumberBuf), "#%07u", frameIndex);

	cairo_set_font_size(cr, FONTSIZE_FRAME_LABEL);
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

void drawAccelerometerData(cairo_t *cr, int32_t *frame)
{
	int16_t accSmooth[3];
	attitude_t attitude;
	t_fp_vector acceleration;
	float magnitude;
	static float lastAccel = 0;

	char labelBuf[32];

	if (flightLog->acc_1G && idents.hasAccs) {
		for (int axis = 0; axis < 3; axis++)
			accSmooth[axis] = frame[idents.accFields[axis]];

		attitude.roll = intToFloat(frame[idents.roll]);
		attitude.pitch = intToFloat(frame[idents.pitch]);
		attitude.heading = intToFloat(frame[idents.heading]);

		//Need to calculate acc in earth frame in order to subtract the 1G of gravity from the result
		acceleration = calculateAccelerationInEarthFrame(accSmooth, &attitude, flightLog->acc_1G);

		//Now that G has been subtracted, work out the length of the acceleration vector
		acceleration.V.X /= flightLog->acc_1G;
		acceleration.V.Y /= flightLog->acc_1G;
		acceleration.V.Z /= flightLog->acc_1G;

		magnitude = sqrt(acceleration.V.X * acceleration.V.X + acceleration.V.Y * acceleration.V.Y + acceleration.V.Z * acceleration.V.Z);

		//Weighted moving average with the recent history to smooth out noise
		lastAccel = (lastAccel * 2 + magnitude) / 3;

		snprintf(labelBuf, sizeof(labelBuf), "Acceleration %.2fG", lastAccel);

		cairo_set_font_size(cr, FONTSIZE_FRAME_LABEL);
		cairo_set_source_rgba(cr, 1, 1, 1, 0.65);

		cairo_move_to(cr, 8, options.imageHeight - 8);
		cairo_show_text(cr, labelBuf);
	}
}

void* pngRenderThread(void *arg)
{
	char filename[255];
	pngRenderingTask_t *task = (pngRenderingTask_t *) arg;

    snprintf(filename, sizeof(filename), "%s%02d.%06d.png", options.outputPrefix, task->outputLogIndex + 1, task->outputFrameIndex);
    cairo_surface_write_to_png (task->surface, filename);
    cairo_surface_destroy (task->surface);

    //Release our slot in the rendering pool, we're done
    semaphore_signal(&pngRenderingSem);

    return 0;
}

/**
 * PNG encoding is so slow and so easily run in parallel, so save the frames using this function
 * (which'll use extra threads to do the work). Be sure to call waitForFramesToSave() before
 * the program ends.
 */
void saveSurfaceAsync(cairo_surface_t *surface, int logIndex, int outputFrameIndex)
{
	if (!pngRenderingSemCreated) {
		semaphore_create(&pngRenderingSem, PNG_RENDERING_THREADS);
		pngRenderingSemCreated = true;
	}

    thread_t thread;
    pngRenderingTask_t *task = (pngRenderingTask_t*) malloc(sizeof(*task));

    task->surface = surface;
    task->outputLogIndex = logIndex;
    task->outputFrameIndex = outputFrameIndex;

    // Reserve a slot in the rendering pool...
    semaphore_wait(&pngRenderingSem);

    thread_create(&thread, pngRenderThread, task);
}

void waitForFramesToSave()
{
	int i;

	if (pngRenderingSemCreated) {
		for (i = 0; i < PNG_RENDERING_THREADS; i++) {
			semaphore_wait(&pngRenderingSem);
		}
	}
}

void renderAnimation(uint32_t startFrame, uint32_t endFrame)
{
	//Change how much data is displayed at one time
	const int windowWidthMicros = 1000 * 1000;

	//Bring the current time into the center of the plot
	const int startXTimeOffset = windowWidthMicros / 2;

	int i;

	int64_t logStartTime = flightLog->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME];
	int64_t logEndTime = flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_TIME];
	int64_t logDurationMicro = logEndTime - logStartTime;

	uint32_t outputFrames;

	int32_t frameValues[FLIGHT_LOG_MAX_FIELDS];
	uint64_t lastCenterTime;
	int64_t frameTime;

	FT_Face ft_face;
	cairo_font_face_t *cairo_face;

	struct craftDrawingParameters_t craftParameters;

	if (endFrame == (uint32_t) -1) {
		endFrame = (logDurationMicro * options.fps + (1000000 - 1)) / 1000000;
	}
	outputFrames = endFrame - startFrame;

	if (FT_New_Memory_Face(freetypeLibrary, (const FT_Byte*)SourceSansPro_Regular_otf, SourceSansPro_Regular_otf_len, 0, &ft_face)) {
		fprintf(stderr, "Failed to load font file\n");
		exit(-1);
	}
	cairo_face = cairo_ft_font_face_create_for_ft_face(ft_face, 0);

	decideCraftParameters(&craftParameters, options.imageWidth, options.imageHeight);

	//Exaggerate values around the origin and compress values near the edges:
	pitchStickCurve = expoCurveCreate(0, 0.700, 500 * (flightLog->rcRate ? flightLog->rcRate : 100) / 100, 1.0, 10);

	gyroCurve = expoCurveCreate(0, 0.666, 500, 1.0, 10);
	accCurve = expoCurveCreate(0, 0.7, 5000, 1.0, 10);
	pidCurve = expoCurveCreate(0, 0.7, 500, 1.0, 10);

	motorCurve = expoCurveCreate(-(flightLog->maxthrottle + flightLog->minthrottle) / 2, 1.0,
			(flightLog->maxthrottle - flightLog->minthrottle) / 2, 1.0, 2);

	int durationSecs = (outputFrames + (options.fps - 1)) / (options.fps);
	int durationMins = durationSecs / 60;
	durationSecs %= 60;

	fprintf(stderr, "%d frames to be rendered at %d FPS [%d:%02d]\n", outputFrames, options.fps, durationMins, durationSecs);
	fprintf(stderr, "\n");

	for (uint32_t outputFrameIndex = startFrame; outputFrameIndex < endFrame; outputFrameIndex++) {
		int64_t windowCenterTime = logStartTime + ((int64_t) outputFrameIndex * 1000000) / options.fps;
		int64_t windowStartTime = windowCenterTime - startXTimeOffset;
		int64_t windowEndTime = windowStartTime + windowWidthMicros;

	    cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, options.imageWidth, options.imageHeight);
	    cairo_t *cr = cairo_create(surface);

		// Find the frame just to the left of the first pixel so we can start drawing lines from there
		int firstFrameIndex = datapointsFindFrameAtTime(points, windowStartTime - 1);

		if (firstFrameIndex == -1) {
			firstFrameIndex = 0;
		}

		cairo_set_font_face(cr, cairo_face);

	    //Plot the upper motor graph
	    if (options.plotMotors) {
			cairo_save(cr);
			{
				if (options.plotPids) {
					//Move up a little bit to make room for the pid graphs
					cairo_translate(cr, 0, options.imageHeight * 0.15);
				} else {
					cairo_translate(cr, 0, options.imageHeight * 0.25);
				}

				drawCenterline(cr);

				cairo_set_line_width(cr, 2.5);

				for (i = 0; i < idents.numMotors; i++) {
					plotLine(cr, idents.motorColors[i], windowStartTime,
							windowEndTime, firstFrameIndex, idents.motorFields[i],
							motorCurve, options.imageHeight * (options.plotPids ? 0.15 : 0.20));
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
								windowEndTime, firstFrameIndex, idents.axisPIDFields[pidType][axis], pidCurve, options.imageHeight * 0.15);

						cairo_set_dash(cr, 0, 0, 0);
					}

					if (options.plotGyros) {
						cairo_set_dash(cr, DOTTED_LINE, DOTTED_LINE_NUM_POINTS, 0);
						cairo_set_line_width(cr, 2);

						plotLine(cr, idents.gyroColors[axis], windowStartTime,
								windowEndTime, firstFrameIndex, idents.gyroFields[axis], gyroCurve, options.imageHeight * 0.15);

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
							windowEndTime, firstFrameIndex, idents.gyroFields[axis], gyroCurve, options.imageHeight * 0.25);

					/*plotLine(cr, idents.gyroColors[axis], windowStartTime,
							windowEndTime, firstFrameIndex, idents.accFields[axis], accCurve, options.imageHeight * 0.25);*/
				}

				drawAxisLabel(cr, "Gyro");
			}

			//Plot other misc fields
			/*for (i = 0; i < idents.numMisc; i++) {
				plotLine(cr, idents.miscColors[i], windowStartTime,
						windowEndTime, firstFrameIndex, idents.miscFields[i], 0, 400, options.imageHeight / 4,
						false);
			}*/
		}
		cairo_restore(cr);

		//Draw a bar highlighting the current time
		double centerX = options.imageWidth / 2.0;

		cairo_set_source_rgba(cr, 1, 0.25, 0.25, 0.2);
		cairo_set_line_width(cr, 20);

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

			drawAccelerometerData(cr, frameValues);
		}

		drawFrameLabel(cr, centerFrameIndex > -1 ? centerFrameIndex : 0, windowCenterTime - flightLog->stats.fieldMinimum[FLIGHT_LOG_FIELD_INDEX_TIME]);

	    cairo_destroy(cr);

		lastCenterTime = windowCenterTime;

		saveSurfaceAsync(surface, selectedLogIndex, outputFrameIndex);

		uint32_t frameWrittenCount = outputFrameIndex - startFrame + 1;
	    if (frameWrittenCount % 500 == 0 || frameWrittenCount == outputFrames) {
			fprintf(stderr, "Rendered %d frames (%.1f%%)%s\n",
				frameWrittenCount, (double)frameWrittenCount / outputFrames * 100,
				frameWrittenCount < outputFrames ? "..." : ".");
	    }
	}

	waitForFramesToSave();
}

void printUsage(const char *argv0)
{
	fprintf(stderr,
		"Blackbox flight log renderer by Nicholas Sherlock\n\n"
		"Usage:\n"
		"     %s [options] <logfilename.txt>\n\n"
		"Options:\n"
		"   --help                 This page\n"
		"   --index <num>          Choose which log from the file should be rendered\n"
		"   --width <px>           Choose the width of the image (default %d)\n"
		"   --height <px>          Choose the height of the image (default %d)\n"
		"   --fps                  FPS of the resulting video (default %d)\n"
		"   --prefix <filename>    Set the prefix of the output frame filenames\n"
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
		"   --prop-style <name>    Style of propeller display (pie/blades, default %s)\n"
		"\n", argv0, defaultOptions.imageWidth, defaultOptions.imageHeight, defaultOptions.fps, defaultOptions.pidSmoothing,
			defaultOptions.gyroSmoothing, defaultOptions.motorSmoothing, defaultOptions.propStyle == PROP_STYLE_BLADES ? "blades" : "pie"
	);
}

/**
 * Parse a time formatted as "%d" (secs) or "%d:%d" (mins:secs) into a count of seconds and
 * store it into frameTime.
 *
 * If the text could not be parsed, false is returned and frameTime is not modified.
 */
bool parseFrameTime(const char *text, uint32_t *frameTime)
{
	const char *cur, *colon = 0, *end;

	for (cur = text; *cur; cur++) {
		if (*cur == ':') {
			if (colon)
				return false;
			else
				colon = cur;
		} else if (!(*cur >= '0' && *cur <= '9'))
			return false;
	}

	end = cur;

	//Check that there are digits before and after the colon and the string wasn't empty...
	if (end == text || (colon && (colon == text || colon == end - 1)))
		return false;

	if (colon) {
		int mins = atoi(text);
		int secs = atoi(colon + 1);

		*frameTime = mins * 60 + secs;
	} else {
		*frameTime = atoi(text);
	}

	return true;
}

void parseCommandlineOptions(int argc, char **argv)
{
	int option_index = 0;
	int c;

	memcpy(&options, &defaultOptions, sizeof(options));

	while (1)
	{
		static struct option long_options[] = {
			{"help", no_argument, &options.help, 1},
			{"index", required_argument, 0, 'i'},
			{"width", required_argument, 0, 'w'},
			{"height", required_argument, 0, 'h'},
			{"fps", required_argument, 0, 'f'},
			{"prefix", required_argument, 0, 'x'},
			{"start", required_argument, 0, 'b'},
			{"end", required_argument, 0, 'e'},
			{"plot-pid", no_argument, &options.plotPids, 1},
			{"plot-gyro", no_argument, &options.plotGyros, 1},
			{"plot-motor", no_argument, &options.plotMotors, 1},
			{"no-plot-pid", no_argument, &options.plotPids, 0},
			{"no-plot-gyro", no_argument, &options.plotGyros, 0},
			{"no-plot-motor", no_argument, &options.plotMotors, 0},
			{"draw-pid-table", no_argument, &options.drawPidTable, 1},
			{"draw-craft", no_argument, &options.drawCraft, 1},
			{"draw-sticks", no_argument, &options.drawSticks, 1},
			{"no-draw-pid-table", no_argument, &options.drawPidTable, 0},
			{"no-draw-craft", no_argument, &options.drawCraft, 0},
			{"no-draw-sticks", no_argument, &options.drawSticks, 0},
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
				if (!parseFrameTime(optarg, &options.timeStart))  {
					fprintf(stderr, "Bad --start time value\n");
					exit(-1);
				}
			break;
			case 'e':
				if (!parseFrameTime(optarg, &options.timeEnd))  {
					fprintf(stderr, "Bad --end time value\n");
					exit(-1);
				}
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
			case 'x':
				options.outputPrefix = optarg;
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
			case '\0':
				//Longopt which has set a flag
			break;
		    case ':':
		        fprintf(stderr, "%s: option '%s' requires an argument\n", argv[0], argv[optind-1]);
		        exit(-1);
			break;
		    default:
		        if (optopt == 0)
		        	fprintf(stderr, "%s: option '%s' is invalid\n", argv[0], argv[optind-1]);
				else
					fprintf(stderr, "%s: option '-%c' is invalid\n", argv[0], optopt);

		        exit(-1);
			break;
		}
	}

	if (optind < argc) {
		options.filename = argv[optind];

		//If the user didn't supply an output filename prefix, create our own based on the input filename
		if (!options.outputPrefix) {
			// Try replacing the file extension with our suffix:
			char *fileExtensionPeriod = strrchr(options.filename, '.');
			int sourceLen;

			if (fileExtensionPeriod) {
				sourceLen = fileExtensionPeriod - options.filename;
			} else {
				//If the original had no extension, just append:
				sourceLen = strlen(options.filename);
			}

			options.outputPrefix = malloc((sourceLen + 2 /*Room for an extra period and the null terminator*/) * sizeof(*options.outputPrefix));

			for (int i = 0; i < sourceLen; i++)
				options.outputPrefix[i] = options.filename[i];

			options.outputPrefix[sourceLen] = '.';
			options.outputPrefix[sourceLen + 1] = '\0';
		}
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

		//Smooth the synthetic PID sum field too
		for (int axis = 0; axis < 3; axis++)
			datapointsSmoothField(points, idents.axisPIDSum[axis], options.pidSmoothing);
	}

	if (options.motorSmoothing) {
		for (int motor = 0; motor < idents.numMotors; motor++)
			datapointsSmoothField(points, idents.motorFields[motor], options.motorSmoothing);
	}
}

void computeExtraFields() {
	int16_t accSmooth[3], gyroData[3];
	int64_t frameTime;
	int32_t frameIndex;
	int32_t frame[FLIGHT_LOG_MAX_FIELDS];
	attitude_t attitude;

	imuInit();

 	if (idents.hasAccs && flightLog->acc_1G) {
		for (frameIndex = 0; frameIndex < points->frameCount; frameIndex++) {
			if (datapointsGetFrameAtIndex(points, frameIndex, &frameTime, frame)) {
				for (int axis = 0; axis < 3; axis++) {
					accSmooth[axis] = frame[idents.accFields[axis]];
					gyroData[axis] = frame[idents.gyroFields[axis]];
				}

				getEstimatedAttitude(gyroData, accSmooth, frameTime, flightLog->acc_1G, flightLog->gyroScale, &attitude);

				//Pack those floats into signed ints to store into the datapoints array:
				datapointsSetFieldAtIndex(points, frameIndex, idents.roll, floatToInt(attitude.roll));
				datapointsSetFieldAtIndex(points, frameIndex, idents.pitch, floatToInt(attitude.pitch));
				datapointsSetFieldAtIndex(points, frameIndex, idents.heading, floatToInt(attitude.heading));
			}
		}
	}

	if (idents.hasPIDs) {
		for (frameIndex = 0; frameIndex < points->frameCount; frameIndex++) {
			if (datapointsGetFrameAtIndex(points, frameIndex, &frameTime, frame)) {
				for (int axis = 0; axis < 3; axis++) {
					int32_t pidSum = frame[idents.axisPIDFields[PID_P][axis]] + frame[idents.axisPIDFields[PID_I][axis]] - frame[idents.axisPIDFields[PID_D][axis]];

					datapointsSetFieldAtIndex(points, frameIndex, idents.axisPIDSum[axis], pidSum);
				}
			}
		}
	}
}

int chooseLog(flightLog_t *log)
{
	if (!log || log->logCount == 0) {
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
	char **fieldNames;
	uint32_t frameStart, frameEnd;
	int fd;

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

    FT_Init_FreeType(&freetypeLibrary);

    flightLog = flightLogCreate(fd);

    selectedLogIndex = chooseLog(flightLog);

    if (selectedLogIndex == -1)
    	return -1;

	//First check out how many frames we need to store so we can pre-allocate (parsing will update the flightlog stats which contain that info)
	flightLogParse(flightLog, selectedLogIndex, 0, 0, false);

	/* Configure our data points array.
	 *
	 * Don't include the leading time or field index fields in the field names / counts, but add on fields
	 * that we'll synthesize
	 */
	fieldNames = malloc(sizeof(*fieldNames) * (flightLog->mainFieldCount - 2 + DATAPOINTS_EXTRA_COMPUTED_FIELDS));

	for (int i = 2; i < flightLog->mainFieldCount; i++) {
		fieldNames[i - 2] = strdup(flightLog->mainFieldNames[i]);
	}

	fieldNames[flightLog->mainFieldCount - 2] = strdup("roll");
	fieldNames[flightLog->mainFieldCount - 1] = strdup("pitch");
	fieldNames[flightLog->mainFieldCount + 0] = strdup("heading");
	fieldNames[flightLog->mainFieldCount + 1] = strdup("axisPID[0]");
	fieldNames[flightLog->mainFieldCount + 2] = strdup("axisPID[1]");
	fieldNames[flightLog->mainFieldCount + 3] = strdup("axisPID[2]");

	points = datapointsCreate(flightLog->mainFieldCount - 2 + DATAPOINTS_EXTRA_COMPUTED_FIELDS, fieldNames, flightLog->stats.fieldMaximum[FLIGHT_LOG_FIELD_INDEX_ITERATION] + 1);

	//Now decode the flight log into the points array
	flightLogParse(flightLog, selectedLogIndex, 0, loadFrameIntoPoints, false);

	identifyFields();

	computeExtraFields();

	applySmoothing();

	frameStart = options.timeStart * options.fps;

	if (options.timeEnd == 0)
		frameEnd = (uint32_t) -1;
	else {
		frameEnd = options.timeEnd * options.fps;
	}

	if (frameEnd <= frameStart) {
		fprintf(stderr, "Error: Selected end time would make this video zero frames long.\n");
		return -1;
	}

	renderAnimation(frameStart, frameEnd);

    return 0;
}
