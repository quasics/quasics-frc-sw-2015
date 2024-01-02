// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "Robot.h"
#include "commands/ArcadeDriveCommand.h"
#include "subsystems/RealDriveBase.h"
#include "subsystems/SimulatedDriveBase.h"
#include "utils/DeadBandEnforcer.h"

RobotContainer::RobotContainer() {
  if (Robot::IsReal()) {
    // Configure real drive base
    m_drivebase.reset(new RealDriveBase);
  } else {
    // Configure simulation drive base
    m_drivebase.reset(new SimulatedDriveBase);
  }
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc2::FunctionalCommand lightingExample(
      // onInit
      [&]() { m_lighting.setSolidStripColor(Lighting::GREEN); },
      // onExecute
      []() {},
      // onEnd
      [&](bool) { m_lighting.setSolidStripColor(Lighting::WHITE); },
      // isFinished
      []() { return false; },
      // Requirements
      {&m_lighting});
  m_lighting.SetDefaultCommand(std::move(lightingExample));

  // Set up arcade drive command as a default.
  const bool isReal = frc::RobotBase::IsReal();
  static DeadBandEnforcer stickDeadBand(-0.01);
  ArcadeDriveCommand::PercentSupplier forwardSupplier =
      // Note: "=" says automatically capture (copies of) any variables
      // referenced; "this" needs to be explicitly captured.
      [=, this]() {
        return stickDeadBand(isReal ? m_controller.GetLeftX()
                                    : m_controller.GetRawAxis(0));
      };
  ArcadeDriveCommand::PercentSupplier rotationSupplier = [=, this]() {
    return stickDeadBand(isReal ? m_controller.GetRightX()
                                : m_controller.GetRawAxis(1));
  };
  ArcadeDriveCommand arcadeDrive(*m_drivebase, forwardSupplier,
                                 rotationSupplier);
  m_drivebase->SetDefaultCommand(std::move(arcadeDrive));

  frc::SmartDashboard::PutData(
      "Toggle logging", new frc2::InstantCommand([] {
        IDrivebase::enableLogging(!IDrivebase::isLoggingEnabled());
      }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
