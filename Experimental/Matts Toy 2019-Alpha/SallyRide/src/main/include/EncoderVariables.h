#ifndef ROBOT_VARIABLES_H_
#define ROBOT_VARIABLES_H_

#include <cmath>

#ifndef M_PI
#define M_PI (std::atan(1)*4)
#endif

#define DRIVE_TRAIN_TICKS_PER_REVOLUTION	360.0f // resolution of E4T (in cycles per revolution, NOT pulses/rev!!!)

#define WHEEL_DIAMETER_INCHES				6
#define WHEEL_CIRCUMFERENCE_INCHES			(WHEEL_DIAMETER_INCHES * M_PI)

#define DRIVE_TRAIN_INCHES_PER_TICK 		(WHEEL_CIRCUMFERENCE_INCHES/DRIVE_TRAIN_TICKS_PER_REVOLUTION)

#endif
