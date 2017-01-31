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
#include "Commands/MoveInSquare.h"
#include "Commands/Arman.h"
#include "Commands/MoveForTime.h"
#include "Commands/PointTurnForAngle.h"
#include "Commands/threeSecondIntake.h"
#include "Commands/GearAuto.h"
#include "Commands/IntakeAuto.h"

#include "SmartDashboard/SmartDashboard.h"
#include "Commands/MoveForTime.h"

OI::OI() {
    // Process operator interface input here.
    auxStick.reset(new Joystick(AuxStickPort));
    driveStick.reset(new Joystick(DriverStickPort));
    
    //Smart Dashboard Buttons
    SmartDashboard::PutData("Robot moves in a square", new MoveInSquare);
    SmartDashboard::PutData("Robot Moves in Arman's initials", new Arman);
    SmartDashboard::PutData("Robot does a point turn", new PointTurnForAngle(90,.3));
    SmartDashboard::PutData("Robot moves for 3 seconds", new MoveForTime(3,.5));
    SmartDashboard::PutData("Intake works for 3 seconds", new threeSecondIntake(3,.5));
    SmartDashboard::PutData("Servo", new GearAuto(.5));
    SmartDashboard::PutData("IntakeAuto works for 3 seconds", new IntakeAuto(.3));
}

std::shared_ptr<Joystick> OI::getDriveStick() {
   return driveStick;
}

std::shared_ptr<Joystick> OI::getAuxStick() {
   return auxStick;
}
