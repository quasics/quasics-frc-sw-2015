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
  AddRequirements(m_drivebase);

  wpi::StringMap<nt::Value> speedSliderProperties{
      {"min", nt::Value::MakeDouble(-1.0)},
      {"max", nt::Value::MakeDouble(+1.0)},
      {"Block increment", nt::Value::MakeDouble(0.01)}};

  auto& tab = frc::Shuffleboard::GetTab(NetworkTableNames::kSettingsTab);

#if 0 // BROKEN_IN_BETA_2
  // The following code is broken when built under Beta 2.
  // 
  // Bug filed: https://github.com/wpilibsuite/BetaTest/issues/80
  m_drivebaseSpeedSlider =
      tab.AddPersistent(NetworkTableNames::kDrivebaseSpeedSliderName,
                        initialDrivebaseSpeedPercent)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties(speedSliderProperties)
          .GetEntry();
#endif
}

void DriveTuningCommand::UpdatePower() {
  if (m_drivebaseSpeedSlider) {
    m_drivebase->SetMotorPower(m_drivebaseSpeedSlider.GetDouble(0),
                              m_drivebaseSpeedSlider.GetDouble(0));
  }
}

// Called when the command is initially scheduled.
void DriveTuningCommand::Initialize() {
  UpdatePower();
}

// Called repeatedly when this Command is scheduled to run
void DriveTuningCommand::Execute() {
  UpdatePower();
}

// Called once the command ends or is interrupted.
void DriveTuningCommand::End(bool interrupted) {
  m_drivebase->Stop();
}