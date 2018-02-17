// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include <Commands/LinearSlideControl.h>
#include "OI.h"
#include "ControllerDefinitions.h"
#include "Commands/Teleop/Teleop.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutoCrossTheLineCommand.h"
#include "Commands/AutoModeScoringCommand.h"
#include "Commands/AutoScoreOnLeftSwitchCommand.h"
#include "Commands/AutoScoreOnRightSwitchCommand.h"
#include "Commands/AutoScoreOnSwitchCommand.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/Primitives/PointTurnLeft.h"
#include "Commands/FaceYellow.h"
#include "Commands/AutoCrossTheLineCommand.h"
#include "Commands/Primitives/MoveForDistance.h"
#include "EncoderVariables.h"
#include "Commands/Primitives/ShoulderMove.h"
#include "Commands/Winch.h"
#include "Commands/TurnLightOn.h"
#include "Commands/TurnLightOff.h"

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    auxStick.reset(new frc::Joystick(1));
    
    driveStick.reset(new frc::Joystick(0));
    

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Linear Slide Control", new LinearSlideControl());
    frc::SmartDashboard::PutData("Auto Score On Right Switch Command", new AutoScoreOnRightSwitchCommand());
    frc::SmartDashboard::PutData("Auto Score On Left Switch Command", new AutoScoreOnLeftSwitchCommand());
    frc::SmartDashboard::PutData("Auto Score On Switch Command", new AutoScoreOnSwitchCommand());
    frc::SmartDashboard::PutData("Auto Cross The Line Command", new AutoCrossTheLineCommand());
    frc::SmartDashboard::PutData("Auto Mode Scoring Command", new AutoModeScoringCommand());
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    frc::SmartDashboard::PutData("Move For Distance", new MoveForDistance(WHEEL_CIRCUMFERENCE_INCHES, .1));
    frc::SmartDashboard::PutData("Turning", new PointTurnLeft(90, .2));
    frc::SmartDashboard::PutData("Turn an arduino light on", new TurnLightOn());
    frc::SmartDashboard::PutData("Turn an arduino light off", new TurnLightOff());
    frc::SmartDashboard::PutData("winch", new Winch());
#ifdef VISION_TRACK_CUBES
    frc::SmartDashboard::PutData("Facing Yellow", new FaceYellow());
#endif
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getdriveStick() {
   return driveStick;
}

std::shared_ptr<frc::Joystick> OI::getauxStick() {
   return auxStick;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS


bool OI::isScissorLiftSignaled() {
	const bool highBoost = auxStick->GetRawButton(XBox_ButtonY);
	return highBoost;
}

bool OI::isLowBoostSignaled() {
	const bool lowBoost = driveStick->GetRawButton(LogitechGamePad_RightTrigger);
	return lowBoost;
}

bool OI::isHighBoostSignaled() {
	const bool highBoost = driveStick->GetRawButton(LogitechGamePad_LeftTrigger);
	return highBoost;
}

double OI::getLeftTrackPower() {
	return driveStick->GetRawAxis(LogitechGamePad_LeftYAxis);
}

double OI::getRightTrackPower() {
	return driveStick->GetRawAxis(LogitechGamePad_RightYAxis);
}
double OI::getYAxis(){
	return auxStick->GetRawAxis(XBox_RightYAxis);
}

bool OI::isSwitchDirectionSignaled() {
	return driveStick->GetRawButton(LogitechGamePad_XButton);
}
