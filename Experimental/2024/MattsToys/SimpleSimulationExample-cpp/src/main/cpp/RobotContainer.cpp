// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TankDrive.h"
#include "subsystems/RealDriveBase.h"
#include "subsystems/SimulatedDriveBase.h"

RobotContainer::RobotContainer() {
  // Figure out the joystick axes to be used to control driving, and
  // create the right kind of drivebase (simulated/real).
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    leftDriveJoystickAxis = 0;  // On the keyboard, this is the "A" and "D" keys
    rightDriveJoystickAxis =
        1;  // On the keyboard, this is the "W" and "S" keys
    m_drivebase.reset(new SimulatedDriveBase);
  } else {
    // TODO: Figure out the raw axes to be used with a real controller.
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
    m_drivebase.reset(new RealDriveBase);
  }

  // Create the TankDrive command (reading from the controller's joysticks), and
  // set it as the default command for the drive base.
  TankDrive::PercentSupplier leftSupplier = [=, this]() {
    return m_controller.GetRawAxis(leftDriveJoystickAxis);
  };
  TankDrive::PercentSupplier rightSupplier = [=, this]() {
    return m_controller.GetRawAxis(rightDriveJoystickAxis);
  };
  TankDrive tankDrive(*m_drivebase, leftSupplier, rightSupplier);
  m_drivebase->SetDefaultCommand(std::move(tankDrive));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
