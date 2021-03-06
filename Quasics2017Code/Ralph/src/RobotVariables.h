#ifndef ROBOT_VARIABLES_H_
#define ROBOT_VARIABLES_H_

#define Practice_Bot

// CODE_REVIEW(mjh): Document what this file actually does (i.e., why all the constants).
// CODE_REVIEW(mjh): Document the purpose of each of the frickin' constants. :-)

//------------------------Robot Specs--------------------------------------------------------------
extern const int wheelDiameterInches;
extern const int driveTrainTicksPerRevolution;
extern const int inchesPerTick;

extern const double SlowMultiplier;
extern const double MediumMultiplier;
extern const double TurboMultiplier;
//------------------------End Of Robot Specs-------------------------------------------------------
//-------------------------------------------------------------------------------------------------

extern const double AutoTimeOut;

//------------------------Joysticks----------------------------------------------------------------
extern const int DriverStickPort;
extern const int AuxStickPort;
//--------------------Driver Stick-----------------------------------------------------------------
//Buttons
extern const int AButtonL;
extern const int BButton;
extern const int XButton;
extern const int YButton;
extern const int LeftShoulder;
extern const int RightShoulder;
extern const int LeftTrigger;
extern const int RightTrigger;
extern const int LeftStickPress;
extern const int RightStickPress;
extern const int StartButton;
extern const int SelectButton;
//Axes
extern const int LeftYAxis;
extern const int LeftXAxis;
extern const int RightYAxis;
extern const int RightXAxis;
//--------------------Co-Pilot Stick---------------------------------------------------------------
//Buttons
extern const int GearActuatorButton;
extern const int OutputButton;
extern const int IntakeButton;
extern const int ClimberButton;
//Axes
extern const int AuxLeftYAxis;
extern const int AuxLeftXAxis;
extern const int AuxRightYAxis;
extern const int AuxRightXAxis;
//------------------------End Joysticks------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
#endif
