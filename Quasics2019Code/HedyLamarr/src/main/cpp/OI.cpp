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

#include <iostream>
#include "ControllerDefinitions.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/TankDrive.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
//#include "Commands/AdjustElevatorStage.h"
//#include "Commands/ElevatorSingleStageTest.h"
#include "Commands/IntakeControl.h"
//#include "Commands/MoveToBottom.h"
//#include "Commands/MoveToPositionOne.h"
//#include "Commands/MoveToPositionTwo.h"
//#include "Commands/MoveToTop.h"
//#include "Commands/OnlyElevator.h"
//#include "Commands/OnlyLifter.h"
//#include "Commands/FullElevatorControl.h"
//#include "Commands/TestLimitSwitches.h"
#include "Commands/AutoLighting.h"

#include "Commands/NewElevatorToBottom.h"
#include "Commands/NewElevatorToLow.h"
#include "Commands/NewElevatorToMiddle.h"
#include "Commands/NewElevatorToTop.h"
#include "Commands/TestHardStops.h"
OI::OI() {
  // Process operator interface input here.
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    operatorStick.reset(new frc::Joystick(1));
    
    driveStick.reset(new frc::Joystick(0));
    

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Tank Drive", new TankDrive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

  frc::SmartDashboard::PutData("Intake", new IntakeControl());
  frc::SmartDashboard::PutData("To Bottom", new NewElevatorToBottom());
  frc::SmartDashboard::PutData("To Low", new NewElevatorToLow());
  frc::SmartDashboard::PutData("To Middle", new NewElevatorToMiddle());
  frc::SmartDashboard::PutData("To Top", new NewElevatorToTop());
  frc::SmartDashboard::PutData("Test Sensors", new TestHardStops());

  //frc::SmartDashboard::PutData("Only Lifter", new OnlyLifter());
  //frc::SmartDashboard::PutData("Only Elevator", new OnlyElevator());

  //frc::SmartDashboard::PutData("Test Lifter",
  //                             new ElevatorSingleStageTest(*Robot::lifter));
  //frc::SmartDashboard::PutData("Test Elevator",
  //                             new ElevatorSingleStageTest(*Robot::elevator));

  //frc::SmartDashboard::PutData("Full Elevator Command",
  //                             new FullElevatorControl());

  //frc::SmartDashboard::PutData("Adjust Lifter",
  //                             new AdjustElevatorSingleStage(*Robot::lifter));
  //frc::SmartDashboard::PutData("Adjust Elevator",
  //                             new AdjustElevatorSingleStage(*Robot::elevator));

  //frc::SmartDashboard::PutData("Test Limit Switches",
  //                             new TestLimitSwitches());

  frc::SmartDashboard::PutData("Test Lighting",
                               new AutoLighting());

  // // maps Left Trigger on the Xbox Controller to moving the Elevator to the Top
  // TopElevatorButton.reset(
  //     new frc::JoystickButton(operatorStick.get(), XBox_LeftTrigger));
  // TopElevatorButton->WhenPressed(new MoveToTop());

  // // maps Right Trigger on the Xbox to moving the Elevator to the Bottom
  // BottomElevatorButton.reset(
  //     new frc::JoystickButton(operatorStick.get(), XBox_RightTrigger));
  // BottomElevatorButton->WhenPressed(new MoveToBottom());

  // // maps Right button on the Xbox Controller to moving the Elevator to Position
  // // One
  // PositionOneElevatorButton.reset(
  //     new frc::JoystickButton(operatorStick.get(), XBox_RightButton));
  // PositionOneElevatorButton->WhenPressed(new MoveToPositionOne());

  // maps Left Button on the Xbox Controller to moving the Elevator to Position
  // Two

  MiddleNewElevatorButton.reset(
      new frc::JoystickButton(operatorStick.get(), XBox_LeftButton));
  MiddleNewElevatorButton->WhenPressed(new NewElevatorToMiddle());

  TopNewElevatorButton.reset(
      new frc::JoystickButton(operatorStick.get(), XBox_LeftTrigger));
  TopNewElevatorButton->WhenPressed(new NewElevatorToTop());

  // maps Right Trigger on the Xbox to moving the Elevator to the Bottom
  BottomNewElevatorButton.reset(
     new frc::JoystickButton(operatorStick.get(), XBox_RightTrigger));
  BottomNewElevatorButton->WhenPressed(new NewElevatorToBottom());

  // maps Right button on the Xbox Controller to moving the Elevator to Position
  // One
  LowNewElevatorButton.reset(
      new frc::JoystickButton(operatorStick.get(), XBox_RightButton));
  LowNewElevatorButton->WhenPressed(new NewElevatorToLow());
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getDriveStick() {
   return driveStick;
}

std::shared_ptr<frc::Joystick> OI::getOperatorStick() {
   return operatorStick;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

// gets the speed of the Left Wheels on the Drive Base
double OI::getLeftTankSpeed() {
  return driveStick->GetRawAxis(LogitechGamePad_LeftYAxis);
}

// gets the speed of the Right Wheels on the Drive Base
double OI::getRightTankSpeed() {
  return driveStick->GetRawAxis(LogitechGamePad_RightYAxis);
}

// tests whether the A Button on the Xbox Controller is pressed to signal the
// Intake
bool OI::isIntakeSignaledPositive() {  // in
  const bool buttonA = operatorStick->GetRawButton(XBox_ButtonA);
  return buttonA;
}

// tests whether the B Button on the Xbox Controller is pressed to signal the
// Intake
bool OI::isIntakeSignaledLowNegative() {  // out, slowly
  const bool buttonB = operatorStick->GetRawButton(XBox_ButtonB);
  return buttonB;
}

// tests whether the Y Button on the Xbox Controller is pressed to signal the
// Intake
bool OI::isIntakeSignaledHighNegative() {  // out, quickly
  const bool buttonY = operatorStick->GetRawButton(XBox_ButtonY);
  return buttonY;
}

// tests whether the Elbow is signaled to go Up
bool OI::isElbowSignaledUp() {
  const double direction = operatorStick->GetRawAxis(XBox_RightYAxis);
  if (direction > 0.3) {
    return true;
  } else {
    return false;
  }
}

// tests whether the Elbow is signaled to go Down
bool OI::isElbowSignaledDown() {
  const double direction = operatorStick->GetRawAxis(XBox_RightYAxis);
  if (direction < -0.3) {
    return true;
  } else {
    return false;
  }
}

// tests the Left Trigger and Right Trigger on the Logitech Controller to see if
// both are pressed for Full Speed
bool OI::isFullSpeedTriggered() {
  // const bool RightTrigger =
  //     driveStick->GetRawButton(LogitechGamePad_RightTrigger);
  const bool LeftTrigger =
      driveStick->GetRawButton(LogitechGamePad_LeftTrigger);
  if (LeftTrigger) {
    return true;
  } else {
    return false;
  }
}

bool OI::isSwitchDriveToggled() {
  const bool LeftShoulder =
      driveStick->GetRawButton(LogitechGamePad_LeftShoulder);
  if(LeftShoulder) {
    return true;
  } else {
    return false;
  }
}

// tests the Right Trigger on the Logitech Controller to see if it is pressed
// for Turtle Speed
bool OI::isTurtleTriggerDown() {
  const bool RightShoulder =
      driveStick->GetRawButton(LogitechGamePad_RightShoulder);
  return RightShoulder;
}

//bool OI::isElevatorToggleDown() {
//  const bool X_button =
//      operatorStick->GetRawButton(XBox_ButtonX);
//  return X_button;
//}

constexpr double kElevatorDeadZoneWidth = 0.3;

//tests the Left Joystick on the Xbox Controller to see if the Elevator is
// signaled to go Up
bool OI::isElevatorMoveUpSignaled() {
  const double joystick = operatorStick->GetRawAxis(XBox_LeftYAxis);
  if (joystick <= -kElevatorDeadZoneWidth) {
    return true;
  } else {
    return false;
  }
}

// tests the Left Joystick on the Xbox Controller to see if the Elevator is
// signaled to go Down
bool OI::isElevatorMoveDownSignaled() {
 const double joystick = operatorStick->GetRawAxis(XBox_LeftYAxis);
  if (joystick >= kElevatorDeadZoneWidth) {
    return true;
  } else {
    return false;
  }
}
