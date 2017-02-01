#ifndef ROBOT_VARIABLES_H_
#define ROBOT_VARIABLES_H_

#include "math.h"

//------------------------Robot Specs--------------------------------------------------------------
extern const int wheelDiameterInches;
extern const int driveTrainTicksPerRevolution;
extern const int inchesPerTick;
//------------------------End Of Robot Specs-------------------------------------------------------
//-------------------------------------------------------------------------------------------------


//------------------------Joysticks----------------------------------------------------------------
#define DriverStickPort 0
#define AuxStickPort 1
//--------------------Driver Stick-----------------------------------------------------------------
//Buttons
#define AButton 2
#define BButton 3
#define XButton 4
#define YButton 1
#define LeftShoulder 5
#define RightShoulder 6
#define LeftTrigger 7
#define RightTrigger 8
#define LeftStickPress 11
#define RightStickPress 12
#define StartButton 9
#define SelectButton 10
//Axes
#define LeftYAxis 1
#define LeftXAxis 0
#define RightYAxis 3
#define RightXAxis 2
//--------------------Co-Pilot Stick---------------------------------------------------------------
//Buttons
#define GearActuatorButton 0
//Axes
#define AuxLeftYAxis 1
#define AuxLeftXAxis 0
#define AuxRightYAxis 3
#define AuxRightXAxis 2
//------------------------End Joysticks------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#endif
