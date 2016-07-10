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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutomaticLighting.h"
#include "Commands/Autonomous.h"
#include "Commands/FlyWheelAutonomous.h"
#include "Commands/FullShooterTeleop.h"
#include "Commands/FullStop.h"
#include "Commands/MoveArmForTime.h"
#include "Commands/MoveForTime.h"
#include "Commands/PistonAutonomous.h"
#include "Commands/SetLightColor.h"
#include "Commands/SetLightDynamics.h"
#include "Commands/ShooterArmTeleop.h"
#include "Commands/ShooterTeleop.h"
#include "Commands/StopArm.h"
#include "Commands/StopDriveBase.h"
#include "Commands/TankDrive.h"
#include "Commands/TurnForTime.h"
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
	// Process operator interface input here.
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	shooterStick.reset(new Joystick(1));

	pilotStick.reset(new Joystick(0));

	// SmartDashboard Buttons
	SmartDashboard::PutData("Full Shooter Teleop", new FullShooterTeleop());
	SmartDashboard::PutData("Full Stop", new FullStop());
	SmartDashboard::PutData("Automatic Lighting", new AutomaticLighting());
	SmartDashboard::PutData("Stop Arm", new StopArm());
	SmartDashboard::PutData("Stop Drive Base", new StopDriveBase());
	SmartDashboard::PutData("Tank Drive", new TankDrive());
	SmartDashboard::PutData("Red Lights", new SetLightColor(Lighting::kRed));
	SmartDashboard::PutData("Green Lights",
			new SetLightColor(Lighting::kGreen));
	SmartDashboard::PutData("Blue Lights", new SetLightColor(Lighting::kBlue));
	SmartDashboard::PutData("White Lights",
			new SetLightColor(Lighting::kWhite));
	SmartDashboard::PutData("Rainbow Lights",
			new SetLightColor(Lighting::kRainbow));
	SmartDashboard::PutData("Lights On", new SetLightDynamics(Lighting::kOn));
	SmartDashboard::PutData("Lights Breathing",
			new SetLightDynamics(Lighting::kBreathing));
	SmartDashboard::PutData("Lights Blinking",
			new SetLightDynamics(Lighting::kBlinking));
	SmartDashboard::PutData("Lights Off", new SetLightDynamics(Lighting::kOff));
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<Joystick> OI::getPilotStick() {
	return pilotStick;
}

std::shared_ptr<Joystick> OI::getShooterStick() {
	return shooterStick;
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
