#include "board.h"
#include "mw.h"

#include "blackbox.h"

static const char blackboxHeader[] =
	"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
	"H Blackbox version:1\n"
	"H Data version:1\n";

/* These headers have info for all 8 motors on them, we'll trim the final fields off to match the number of motors in the mixer: */
static const char * const blackboxHeaderFields[] = {
	"H Field name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisD[2]"
		",rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],gyroData[0],gyroData[1],gyroData[2],motor[0],motor[1],motor[2]"
		",motor[3],motor[4],motor[5],motor[6],motor[7]",

	/* loopIteration, time, throttle and motors values aren't signed */
         "H Field signed:0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0",

	/*
	 * loopIteration merely increments, time advances in a straight line, motors and gyros predict an average of the
	 * last two measurements (to reduce the impact of noise), all others predict the previous frame:
	 */
	"H Field P-predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3",

	// RC fields are encoded together as a group, other fields use signed-VB
     "H Field P-encoding:0,0,0,0,0,0,0,0,0,0,0,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0",

	/*
	 * Throttle and motor[0] are predicted to be minthrottle, the other motors predict to be the same as motor[0].
	 * Other fields have no predictions:
	 */
	"H Field I-predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,4,5,5,5,5,5,5,5",

	// loopIteration, time, throttle and motor[0] are stored with unsigned-VB, the rest with signed-VB:
     "H Field I-encoding:1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0"
};

typedef enum BlackboxState {
	BLACKBOX_STATE_DISABLED = 0,
	BLACKBOX_STATE_STOPPED,
	BLACKBOX_STATE_SEND_HEADER,
	BLACKBOX_STATE_SEND_FIELDINFO,
	BLACKBOX_STATE_SEND_SYSINFO,
	BLACKBOX_STATE_RUNNING
} BlackboxState;

//From mixer.c:
extern uint8_t numberMotor;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;
static uint32_t startTime;
static unsigned int headerXmitIndex;
uint32_t blackboxIteration;

static serialPort_t *blackboxPort;

// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackbox_values_t blackboxHistoryRing[3];

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackbox_values_t* blackboxHistory[3];

// This points into the generation 0 buffer of blackboxHistoryRing, used to give a nice easy interface for MW to poke data in
blackbox_values_t *blackboxCurrent;

static void blackboxWrite(uint8_t value)
{
	serialWrite(blackboxPort, value);
}

static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(blackboxPort, c);
}

//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
static void blackboxPrintf(char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, _putc, fmt, va);
    va_end(va);
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
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

#define FIELD_ZERO  0
#define FIELD_4BIT  1
#define FIELD_8BIT  2
#define FIELD_16BIT 3

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
static void writeTag8_4S16(int32_t *values) {
	uint8_t selector;
	int x;

	/*
	 * 4-bit fields can only be combined with their paired neighbor (there are two pairs), so choose a
	 * larger encoding if that's not possible. This is the most compact representation of this logic
	 * I could come up with:
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

	//Motors can be below minthrottle when disarmed, but that doesn't happen much
	writeUnsignedVB(blackboxCurrent->motor[0] - mcfg.minthrottle);

	//Motors tend to be similar to each other
	for (x = 1; x < numberMotor; x++)
		writeSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);

	//Rotate our history buffers:

	//The current state becomes the new "before" state
	blackboxHistory[1] = blackboxHistory[0];
	//And since we have no other history, we also use it for the "before, before" state
	blackboxHistory[2] = blackboxHistory[0];
	//And advance the current state over to a blank space ready to be filled
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
	blackboxCurrent = blackboxHistory[0];
}

static void writeInterframe(void)
{
	int x;
	int32_t rcDeltas[4];

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
		writeSignedVB(blackboxCurrent->axisI[x] - blackboxLast->axisI[x]);

	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxCurrent->axisD[x] - blackboxLast->axisD[x]);

	for (x = 0; x < 4; x++)
		rcDeltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];

	/*
	 * RC tends to stay the same or very small for many frames at a time, so use an encoding that
	 * can pack multiple values per byte:
	 */
	writeTag8_4S16(rcDeltas);

	//Since gyros and motors are noisy, base the prediction on the average of the history:
	for (x = 0; x < 3; x++)
		writeSignedVB(blackboxHistory[0]->gyroData[x] - (blackboxHistory[1]->gyroData[x] + blackboxHistory[2]->gyroData[x]) / 2);

	for (x = 0; x < numberMotor; x++)
		writeSignedVB(blackboxHistory[0]->motor[x] - (blackboxHistory[1]->motor[x] + blackboxHistory[2]->motor[x]) / 2);

	//Rotate our history buffers
	blackboxHistory[2] = blackboxHistory[1];
	blackboxHistory[1] = blackboxHistory[0];
	blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
	blackboxCurrent = blackboxHistory[0];
}

void startBlackbox(void)
{
	if (blackboxState == BLACKBOX_STATE_STOPPED) {
		if (mcfg.telemetry_port == TELEMETRY_PORT_UART)
			serialInit(115200);

		startTime = millis();
		headerXmitIndex = 0;
		blackboxIteration = 0;
		blackboxState = BLACKBOX_STATE_SEND_HEADER;

		blackboxHistory[0] = &blackboxHistoryRing[0];
		blackboxHistory[1] = &blackboxHistoryRing[1];
		blackboxHistory[2] = &blackboxHistoryRing[2];
		blackboxCurrent = blackboxHistory[0];
	}
}

void finishBlackbox(void)
{
	if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED) {
		blackboxState = BLACKBOX_STATE_STOPPED;

	    if (mcfg.telemetry_port == TELEMETRY_PORT_UART)
	        serialInit(mcfg.serial_baudrate);
	}
}

void handleBlackbox(void)
{
	int i;
	const int SERIAL_CHUNK_SIZE = 16;
	static int charXmitIndex = 0;
	int motorsToRemove, endIndex;

	switch (blackboxState) {
		case BLACKBOX_STATE_SEND_HEADER:
			/*
			 * Once the UART has had time to init, transmit the header in chunks so we don't overflow our transmit
			 * buffer.
			 */
			if (millis() > startTime + 100 /*&& isSerialTransmitBufferEmpty(blackboxPort)*/) {
				for (i = 0; i < SERIAL_CHUNK_SIZE && blackboxHeader[headerXmitIndex] != '\0'; i++, headerXmitIndex++)
					blackboxWrite(blackboxHeader[headerXmitIndex]);

				if (blackboxHeader[headerXmitIndex] == '\0') {
					blackboxState = BLACKBOX_STATE_SEND_FIELDINFO;
					headerXmitIndex = 0;
					charXmitIndex = 0;
				}
			}
		break;
		case BLACKBOX_STATE_SEND_FIELDINFO:
			/*
			 * Once again, chunking up the data so we don't exceed our datarate. This time we're removing the excess field defs
			 * for motors we don't have.
			 */
			motorsToRemove = 8 - numberMotor;

			if (headerXmitIndex < sizeof(blackboxHeaderFields) / sizeof(blackboxHeaderFields[0])){
				endIndex = strlen(blackboxHeaderFields[headerXmitIndex]) - (headerXmitIndex == 0 ? strlen(",motor[x]") : strlen(",x")) * motorsToRemove;

				for (i = charXmitIndex; i < charXmitIndex + SERIAL_CHUNK_SIZE && i < endIndex; i++)
					blackboxWrite(blackboxHeaderFields[headerXmitIndex][i]);

				if (i == endIndex) {
					blackboxWrite('\n');
					headerXmitIndex++;
					charXmitIndex = 0;
				} else {
					charXmitIndex = i;
				}
			} else {
				blackboxState = BLACKBOX_STATE_SEND_SYSINFO;
				headerXmitIndex = 0;
			}
		break;
		case BLACKBOX_STATE_SEND_SYSINFO:

			switch (headerXmitIndex) {
				case 0:
					blackboxPrintf("H rcRate:%d\n", cfg.rcRate8);
				break;
				case 1:
					blackboxPrintf("H minthrottle:%d\n", mcfg.minthrottle);
				break;
				case 2:
					blackboxPrintf("H maxthrottle:%d\n", mcfg.maxthrottle);
				break;
				default:
					blackboxState = BLACKBOX_STATE_RUNNING;
			}

			headerXmitIndex++;
		break;
		case BLACKBOX_STATE_RUNNING:
        	// Copy current system values into the blackbox to be written out
        	blackboxCurrent->time = currentTime;

        	for (i = 0; i < numberMotor; i++)
        		blackboxCurrent->motor[i] = motor[i];

        	for (i = 0; i < 4; i++)
        		blackboxCurrent->rcCommand[i] = rcCommand[i];

        	for (i = 0; i < 3; i++)
        		blackboxCurrent->gyroData[i] = gyroData[i];

			// Write a keyframe every 32 frames so we can resynchronise upon missing frames
			if ((blackboxIteration & 0x1F) == 0)
				writeIntraframe();
			else
				writeInterframe();

			blackboxIteration++;
		break;
		default:
		break;
	}
}

bool canUseBlackboxWithCurrentConfiguration(void)
{
    if (!feature(FEATURE_BLACKBOX))
        return false;

    return true;
}

void initBlackbox(void)
{
    if (canUseBlackboxWithCurrentConfiguration())
    	blackboxState = BLACKBOX_STATE_STOPPED;
    else
    	blackboxState = BLACKBOX_STATE_DISABLED;

    switch (mcfg.blackbox_port) {
    	case TELEMETRY_PORT_UART:
    		blackboxPort = core.mainport;
    	break;
    	default:
        	blackboxState = BLACKBOX_STATE_DISABLED;
    }

    //mw.c needs somewhere to poke its PIDs
    blackboxCurrent = &blackboxHistoryRing[0];
}
