// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#ifndef OI_H
#define OI_H

#include "frc/WPILib.h"

class OI {
 private:
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	
	std::shared_ptr<frc::Joystick> operatorStick;
	std::shared_ptr<frc::Joystick> driveStick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // creates an object for all the elevator buttons
  //std::shared_ptr<frc::JoystickButton> TopElevatorButton;
  //std::shared_ptr<frc::JoystickButton> BottomElevatorButton;
  //std::shared_ptr<frc::JoystickButton> PositionOneElevatorButton;
  //std::shared_ptr<frc::JoystickButton> PositionTwoElevatorButton;

 public:
  OI();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

	std::shared_ptr<frc::Joystick> getDriveStick();
	std::shared_ptr<frc::Joystick> getOperatorStick();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

  double getLeftTankSpeed();
  double getRightTankSpeed();
  double getElbowDirection();

  bool isTurtleTriggerDown();
  bool isElevatorToggleDown();
  bool isFullSpeedTriggered();
  bool isSwitchDriveToggled();

  bool isIntakeSignaledPositive();      // in
  bool isIntakeSignaledLowNegative();   // out, slowly
  bool isIntakeSignaledHighNegative();  // out, quickly

  // Returns true iff operator wants the manipulator to be raised.
  bool isElbowSignaledUp();
  // Returns true iff operator wants the manipulator to be lowered.
  bool isElbowSignaledDown();

  /// Returns true iff the elevator (and lifter) should be moved up.
  bool isElevatorMoveUpSignaled();

  /// Returns true iff the elevator (and lifter) should be moved down.
  bool isElevatorMoveDownSignaled();
};

#endif
