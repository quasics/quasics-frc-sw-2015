// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "Robot.h"
#include "commands/ArcadeDriveCommand.h"
#include "commands/DirectionalLighting.h"
#include "subsystems/RealDriveBase.h"
#include "subsystems/SimulatedDriveBase.h"
#include "utils/DeadBandEnforcer.h"

RobotContainer::RobotContainer() {
  if (Robot::IsReal()) {
    // Configure real drive base
    m_drivebase.reset(new RealDriveBase);
  } else {
    // TODO: Enable switching between "pure simulation" and XRP "little bot".
    // Configure simulation drive base
    m_drivebase.reset(new SimulatedDriveBase);
  }
  ConfigureBindings();
}

void RobotContainer::ConfigureSimpleLightingExample() {
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
}

void RobotContainer::ConfigureDirectionalLighting() {
  DirectionalLighting lightingCommand(m_lighting);
  m_lighting.SetDefaultCommand(std::move(lightingCommand));
}

void RobotContainer::ConfigureBindings() {
#ifdef USE_SIMPLE_LIGHTING_EXAMPLE
  ConfigureSimpleLightingExample();
#else
  ConfigureDirectionalLighting();
#endif

  // Set up arcade drive command as a default.
  const bool isReal = frc::RobotBase::IsReal();
  static DeadBandEnforcer stickDeadBand(0.1);
  ArcadeDriveCommand::PercentSupplier forwardSupplier =
      // Note: "=" says automatically capture (copies of) any variables
      // referenced; "this" needs to be explicitly captured.
      [=, this]() {
        return stickDeadBand(isReal ? m_controller.GetLeftY()
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

  // Commands for SysId profiling.
  static frc2::CommandPtr quasistaticFwd =
      m_drivebase->sysIdQuasistatic(frc2::sysid::kForward);
  static frc2::CommandPtr quasistaticRev =
      m_drivebase->sysIdQuasistatic(frc2::sysid::kReverse);
  static frc2::CommandPtr dynamicFwd =
      m_drivebase->sysIdDynamic(frc2::sysid::kForward);
  static frc2::CommandPtr dynamicRev =
      m_drivebase->sysIdDynamic(frc2::sysid::kReverse);
  frc::SmartDashboard::PutData("Quasistatic Fwd", quasistaticFwd.get());
  frc::SmartDashboard::PutData("Quasistatic Rev", quasistaticRev.get());
  frc::SmartDashboard::PutData("Dynamic Fwd", dynamicFwd.get());
  frc::SmartDashboard::PutData("Dynamic Rev", dynamicRev.get());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
