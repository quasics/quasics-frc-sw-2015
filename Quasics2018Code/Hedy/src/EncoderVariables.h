#ifndef ROBOT_VARIABLES_H_
#define ROBOT_VARIABLES_H_

#include <math.h>

#ifndef M_PI
#define M_PI atan(1)*4
#endif

#define wheelDiameterInches 6
#define driveTrainTicksPerRevolution 360
#define InchesPerTick wheelDiameterInches * M_PI/driveTrainTicksPerRevolution

#endif
