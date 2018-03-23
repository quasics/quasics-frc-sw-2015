// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"
#include "ControllerDefinitions.h"
#include "Commands/FaceTape.h"
#include "Commands/LightingTestCommand.h"
#include "Commands/LinearSlideControl.h"
#include "Commands/Primitives/PointTurnRight.h"
#include "Commands/Primitives/MoveForDistance.h"
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
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    auxStick.reset(new frc::Joystick(1));
    
    driveStick.reset(new frc::Joystick(0));
    

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Linear Slide Control", new LinearSlideControl());


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    frc::SmartDashboard::PutData("winch", new Winch());
    frc::SmartDashboard::PutData("MoveForDistance", new MoveForDistance(120, -.4));

#ifdef VISION_TRACK_CUBES
    frc::SmartDashboard::PutData("Facing Yellow", new FaceYellow());
#else
    frc::SmartDashboard::PutData("Tape", new FaceTape());
    frc::SmartDashboard::PutData("Turn Right", new PointTurnRight(45,.1));
    frc::SmartDashboard::PutData("Test lighting command", new LightingTestCommand());
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

bool OI::isLinearSlideUpSignaled() {
	const bool signaled = auxStick->GetRawButton(XBox_RightButton);
	return signaled;
}

bool OI::isLinearSlideDownSignaled() {
	const bool signaled = auxStick->GetRawButton(XBox_LeftButton);
	return signaled;
}

bool OI::isWinchSignaled() {
	const bool signaled = auxStick->GetRawButton(XBox_ButtonX);
	return signaled;
}

bool OI::isLowBoostSignaled() {
	const bool lowBoost = driveStick->GetRawButton(LogitechGamePad_RightTrigger);
	return lowBoost;
}

double OI::getWinchPower() {
	return auxStick->GetRawAxis(XBox_RightYAxis);
}

bool OI::isCubeIntakeSignaled() {
	const bool buttonB = auxStick->GetRawButton(XBox_ButtonB);
	return buttonB;
}

bool OI::isCubeHighSpeedExhaustSignaled() {
	const bool buttonY = auxStick->GetRawButton(XBox_ButtonY);
	return buttonY;
}

bool OI::isCubeLowSpeedExhaustSignaled() {
	const bool buttonA = auxStick->GetRawButton(XBox_ButtonA);
	return buttonA;
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

bool OI::isSwitchDirectionSignaled() {
	return driveStick->GetRawButton(LogitechGamePad_XButton);
}
