// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/BaseCommands/Autonomous/PointTurnForAngleRaymond.h"
#include "OI.h"
#include "Commands/BaseCommands/Autonomous/MoveForDistance.h"
#include "Commands/CommandGroups/TeleopCommandGroup.h"
#include "Commands/BaseCommands/Autonomous/AutoTurnLeft.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    auxStick.reset(new Joystick(1));
    
    driveStick.reset(new Joystick(0));
    
    SmartDashboard::PutData("move for distance", new MoveForDistance(36, .35));
    SmartDashboard::PutData("Turn 90 Degrees", new PointTurnForAngle(-90, .2));
    SmartDashboard::PutData("Default Teleop", new TeleopCommandGroup ());
    SmartDashboard::PutData("turn for time", new AutoTurnLeft(3, .5));
    // SmartDashboard Buttons
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<Joystick> OI::getDriveStick() {
   return driveStick;
}

std::shared_ptr<Joystick> OI::getAuxStick() {
   return auxStick;
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
