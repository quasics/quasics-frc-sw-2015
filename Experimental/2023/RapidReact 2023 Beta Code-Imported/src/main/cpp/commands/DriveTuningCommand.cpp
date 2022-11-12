// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveTuningCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>

#include "Constants.h"

DriveTuningCommand::DriveTuningCommand(Drivebase* drivebase,
                                       double initialDrivebaseSpeedPercent)
    : m_drivebase(drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
#ifdef READY_FOR_BETA_3
  wpi::StringMap<std::shared_ptr<nt::Value>> speedSliderProperties{
      {"min", nt::Value::MakeDouble(-1.0)},
      {"max", nt::Value::MakeDouble(+1.0)},
      {"Block increment", nt::Value::MakeDouble(0.01)}};

  auto& tab = frc::Shuffleboard::GetTab(NetworkTableNames::kSettingsTab);
  m_drivebaseSpeedSlider =
      tab.AddPersistent(NetworkTableNames::kDrivebaseSpeedSliderName,
                        initialDrivebaseSpeedPercent)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties(speedSliderProperties)
          .GetEntry();
#endif
}

// Called when the command is initially scheduled.
void DriveTuningCommand::Initialize() {
  m_drivebase->SetMotorPower(m_drivebaseSpeedSlider.GetDouble(0),
                             m_drivebaseSpeedSlider.GetDouble(0));
}

// Called repeatedly when this Command is scheduled to run
void DriveTuningCommand::Execute() {
  m_drivebase->SetMotorPower(m_drivebaseSpeedSlider.GetDouble(0),
                             m_drivebaseSpeedSlider.GetDouble(0));
}

// Called once the command ends or is interrupted.
void DriveTuningCommand::End(bool interrupted) {
  m_drivebase->Stop();
}