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
#include "subsystems/XrpDriveBase.h"

/**
 * Iff true, we'll talk to an XRP device while running in the simulator.
 * Otherwise, we'll assuming that we're running *fully* in the simulator, with
 * no actual "robot" hardware being involved.
 */
constexpr bool USE_XRP_UNDER_SIMULATION = false;

RobotContainer::RobotContainer() {
  // Allocate any subsystems that aren't directly embedded in this object.
  allocateDriveBase();

  // Bind default commands to the subsystems.
  setUpTankDrive();
}

void RobotContainer::setUpTankDrive() {
  // Figure out the joystick axes to be used to control driving.  (This assumes
  // that we're using the keyboard controls to simulate a joystick under the
  // simulator, or else a Logitech controller for a real robot.)
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    // On the keyboard:
    // * axis 0 is the "A" (negative)/"D" (positive) keys
    // * axis 1 is the "W" (negative)/"S" (positive) keys
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
  } else {
    leftDriveJoystickAxis = OperatorConstants::LogitechGamePad::LeftYAxis;
    rightDriveJoystickAxis = OperatorConstants::LogitechGamePad::RightYAxis;
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

void RobotContainer::allocateDriveBase() {
  if (frc::RobotBase::IsReal()) {
    // OK, we're running on a "big bot".
    m_drivebase.reset(new RealDriveBase);
  } else {
    // OK, we're running under simulation.  However, this could either mean that
    // we're talking to an XRP "little bot", or doing *pure* (GUI-based)
    // simulation on a PC of some sort.
    if (USE_XRP_UNDER_SIMULATION) {
      m_drivebase.reset(new XrpDriveBase);
    } else {
      m_drivebase.reset(new SimulatedDriveBase);
    }
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
