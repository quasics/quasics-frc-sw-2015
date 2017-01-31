#ifndef ROBOT_VARIABLES_H_
#define ROBOT_VARIABLES_H_

#include "math.h"

//------------------------Wiring Info--------------------------------------------------------------
//------------------------Sensors------------------------------------------------------------------
#define LEFT_ENCODER_A_CHANNEL	0
#define LEFT_ENCODER_B_CHANNEL	1
#define RIGHT_ENCODER_A_CHANNEL	2
#define RIGHT_ENCODER_B_CHANNEL	3
//------------------------Motors-------------------------------------------------------------------
#define LEFT_FRONT_MOTOR_CHANNEL	0
#define LEFT_BACK_MOTOR_CHANNEL		1
#define RIGHT_FRONT_MOTOR_CHANNEL	2
#define RIGHT_BACK_MOTOR_CHANNEL	3
#define INTAKE_MOTOR_CHANNEL		4
//------------------------Other Actuators----------------------------------------------------------
#define GEAR_SERVO_CHANNEL			5
//------------------------End Of Wiring Info ------------------------------------------------------


//------------------------Robot Specs--------------------------------------------------------------
extern const int wheelDiameterInches;
extern const int driveTrainTicksPerRevolution;
extern const int inchesPerTick;
//------------------------End Of Robot Specs-------------------------------------------------------


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
//------------------------End Joysticks------------------------------------------------------------
#endif
