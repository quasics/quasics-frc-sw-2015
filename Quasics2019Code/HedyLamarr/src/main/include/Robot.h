// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#ifndef _ROBOT_H
#define _ROBOT_H

#include "frc/WPILib.h"
#include "frc/commands/Command.h"
#include "frc/livewindow/LiveWindow.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Commands/AutonomousCommand.h"
#include "Subsystems/DriveBase.h"
#include "Subsystems/Elbow.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/Lifter.h"
#include "Subsystems/Lighting.h"
#include "Subsystems/Manipulator.h"

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "OI.h"

class Robot : public frc::TimedRobot {
 public:
  frc::Command* autonomousCommand = nullptr;
  static std::unique_ptr<OI> oi;
  frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
  frc::SendableChooser<frc::Command*> chooser;
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  static std::shared_ptr<DriveBase> driveBase;
  static std::shared_ptr<Manipulator> manipulator;
  static std::shared_ptr<Elevator> elevator;
  static std::shared_ptr<Lifter> lifter;
  static std::shared_ptr<Elbow> elbow;
  static std::shared_ptr<Lighting> lighting;

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  void RobotInit() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
};
#endif
