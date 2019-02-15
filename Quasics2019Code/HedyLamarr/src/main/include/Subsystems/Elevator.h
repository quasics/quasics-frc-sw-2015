// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <frc/WPILib.h>
#include <frc/commands/Subsystem.h>
#include "Subsystems/ElevatorStage.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class Elevator : public ElevatorStage {
 private:
  // It's desirable that everything possible is private except
  // for methods that implement subsystem capabilities
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  std::shared_ptr<frc::DigitalInput> elevatorFirmStopLow;
  std::shared_ptr<frc::DigitalInput> elevatorFirmStopHigh;
  std::shared_ptr<frc::DigitalInput> hallEffectLowElevator;
  std::shared_ptr<frc::DigitalInput> hallEffectHighElevator;
  std::shared_ptr<frc::SpeedController> elevatorMotor;

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
 public:
  Elevator();
  void InitDefaultCommand() override;

  bool atTop() override;
  bool atBottom() override;
  bool atPositionOne() override;
  bool atPositionTwo() override;

  void moveUp() override;
  void moveDown() override;
  void moveSlowlyUp() override;
  void moveSlowlyDown() override;
  void stop() override;

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};

#endif
