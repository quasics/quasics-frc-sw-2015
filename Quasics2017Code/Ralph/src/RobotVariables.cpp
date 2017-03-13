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

const double SlowMultiplier = 0.25;
const double MediumMultiplier = 0.60;
const double TurboMultiplier = 0.70;

const double AutoTimeOut = 10;

//------------------------Joysticks----------------------------------------------------------------
const int DriverStickPort = 0;
const int AuxStickPort = 1;
//--------------------Driver Stick-----------------------------------------------------------------
//Buttons
const int AButtonL = 2;
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
const int GearActuatorButton = 2;
const int OutputButton = 1;
const int IntakeButton = 3;
const int ActuatorButton = 4;

//Axes
const int AuxRightYAxis = 3;
const int AuxRightXAxis = 2;
//------------------------End Joysticks------------------------------------------------------------
