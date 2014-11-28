#ifndef BLACKBOX_FIELDDEFS_H_
#define BLACKBOX_FIELDDEFS_H_

//No prediction:
#define FLIGHT_LOG_FIELD_PREDICTOR_0              0

//Predict that the field is the same as last frame:
#define FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS       1

//Predict that the slope between this field and the previous item is the same as that between the past two history items:
#define FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE  2

//Predict that this field is the same as the average of the last two history items:
#define FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2      3

//Predict that this field is minthrottle
#define FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE    4

//Predict that this field is the same as motor 0
#define FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0        5

//This field always increments
#define FLIGHT_LOG_FIELD_PREDICTOR_INC            6

//Predict this GPS co-ordinate is the GPS home co-ordinate (or no prediction if that coordinate is not set)
#define FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD     7


#define FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       0
#define FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     1
#define FLIGHT_LOG_FIELD_ENCODING_U8              2
#define FLIGHT_LOG_FIELD_ENCODING_U16             3
#define FLIGHT_LOG_FIELD_ENCODING_U32             4
#define FLIGHT_LOG_FIELD_ENCODING_S8              5
#define FLIGHT_LOG_FIELD_ENCODING_S16             6
#define FLIGHT_LOG_FIELD_ENCODING_S32             7
#define FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       8
#define FLIGHT_LOG_FIELD_ENCODING_NULL            9

#endif
