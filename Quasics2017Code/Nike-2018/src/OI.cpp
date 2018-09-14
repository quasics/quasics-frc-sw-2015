// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

//-----------------------------Header Include----------------------------------
#include "OI.h"

//-----------------------------Library Includes--------------------------------
#include "SmartDashboard/SmartDashboard.h"

//-----------------------------Command Includes--------------------------------

#include "Commands/CommandGroups/TeleopCommandGroup.h"

//-----------------------------------------------------------------------------

//#define PRACTICE_COMMANDS

OI::OI() {		//Constructor
	//Reset the joystick pointer
	auxStick.reset(new Joystick(1));
	driveStick.reset(new Joystick(0));

}

std::shared_ptr<Joystick> OI::getDriveStick() {		//Call the drive stick
	return driveStick;
}

std::shared_ptr<Joystick> OI::getAuxStick() {		//Call the auxiliary stick
	return auxStick;
}
