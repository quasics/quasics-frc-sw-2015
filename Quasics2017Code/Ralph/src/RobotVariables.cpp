/*
 * RobotVariables.cpp
 *
 *  Created on: Jan 28, 2017
 *      Author: axf105
 */
#include "RobotVariables.h"

#include <cmath>

#ifndef M_PI
#define M_PI	(std::atan(1)*4)		// Note: Not all compilers define M_PI, though the RoboRio compiler apparently does
#endif

const int wheelDiameterInches = 6;
const int driveTrainTicksPerRevolution = 1440;

const int inchesPerTick = wheelDiameterInches * M_PI/driveTrainTicksPerRevolution;

const double SlowMultiplier = 0.65;
const double MediumMultiplier = 0.80;
const double TurboMultiplier = 0.90;

//------------------------Joysticks----------------------------------------------------------------
const int DriverStickPort = 0;
const int AuxStickPort = 1;
//--------------------Driver Stick-----------------------------------------------------------------
//Buttons
const int AButton = 2;
const int BButton = 3;
const int XButton = 4;
const int YButton = 1;
const int LeftShoulder = 5;
const int RightShoulder = 6;
const int LeftTrigger = 7;
const int RightTrigger = 8;
const int LeftStickPress = 11;
const int RightStickPress = 12;
const int StartButton = 9;
const int SelectButton = 10;
//Axes
const int LeftYAxis = 1;
const int LeftXAxis = 0;
const int RightYAxis = 3;
const int RightXAxis = 2;
//--------------------Co-Pilot Stick---------------------------------------------------------------
//Buttons
const int GearActuatorButton = 0;
const int OutputButton = 6;
const int IntakeButton = 5;
//Axes
const int AuxLeftYAxis = 1;
const int AuxLeftXAxis = 0;
const int AuxRightYAxis = 3;
const int AuxRightXAxis = 2;
//------------------------End Joysticks------------------------------------------------------------
