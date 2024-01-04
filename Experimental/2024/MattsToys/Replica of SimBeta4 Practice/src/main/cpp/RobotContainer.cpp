#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "Robot.h"
#include "commands/ArcadeDriveCommand.h"
#include "subsystems/RealDriveBase.h"
#include "subsystems/SimulatedDriveBase.h"
#include "utils/DeadBandEnforcer.h"

//checks which one are you

RobotContainer::RobotContainer() {
  if (Robot::IsReal()) {
    m_drivebase.reset(new RealDriveBase);
  } else {
    m_drivebase.reset(new SimulatedDriveBase);
  }
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc2::FunctionalCommand lightingExample(
      [&]() { m_lighting.setSolidStripColor(Lighting::GREEN); },
      []() {},
      [&](bool) { m_lighting.setSolidStripColor(Lighting::WHITE); },
      []() { return false; },
      {&m_lighting});
  m_lighting.SetDefaultCommand(std::move(lightingExample));

//Here is the arcade drive implementation
//Ask about clarification on equal sign

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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
