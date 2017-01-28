/*
 * RobotVariables.cpp
 *
 *  Created on: Jan 28, 2017
 *      Author: axf105
 */
#include "RobotVariables.h"

const int wheelDiameterInches = 6;
const int driveTrainTicksPerRevolution = 1440;

const int inchesPerTick = wheelDiameterInches * atan(1)*4/driveTrainTicksPerRevolution;
