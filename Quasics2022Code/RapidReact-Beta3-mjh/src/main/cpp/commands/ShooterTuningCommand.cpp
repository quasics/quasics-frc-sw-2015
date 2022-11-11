// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterTuningCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>

#include "Constants.h"

ShooterTuningCommand::ShooterTuningCommand(Shooter* shooter,
                                           double initialShooterSpeedPercent,
                                           double initialRollerSpeedPercent)
    : m_shooter(shooter) {
  AddRequirements(m_shooter);

#if 0 // BROKEN_IN_BETA_2
  // The following code is broken when built under Beta 2.
  // 
  // Bug filed: https://github.com/wpilibsuite/BetaTest/issues/80
  wpi::StringMap<std::shared_ptr<nt::Value>> speedSliderProperties{
      {"min", nt::Value::MakeDouble(-1.0)},
      {"max", nt::Value::MakeDouble(+1.0)},
      {"Block increment", nt::Value::MakeDouble(0.01)}};

  auto& tab = frc::Shuffleboard::GetTab(NetworkTableNames::kSettingsTab);
  m_shooterSpeedSlider =
      tab.AddPersistent(NetworkTableNames::kShooterSpeedSliderName,
                        initialShooterSpeedPercent)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties(props2)
          .GetEntry();

  m_rollerSpeedSlider =
      tab.AddPersistent(NetworkTableNames::kRollerSpeedSliderName,
                        initialRollerSpeedPercent)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties(speedSliderProperties)
          .GetEntry();
#endif
}

void ShooterTuningCommand::UpdateSpeeds() {
  if (m_shooterSpeedSlider) {
    m_shooter->SetFlywheelSpeed(m_shooterSpeedSlider.GetDouble(0));
  }
  if (m_rollerSpeedSlider) {
    m_shooter->SetRollerSpeed(m_rollerSpeedSlider.GetDouble(0));
  }
}

// Called when the command is initially scheduled.
void ShooterTuningCommand::Initialize() {
  UpdateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void ShooterTuningCommand::Execute() {
  UpdateSpeeds();
}

// Called once the command ends or is interrupted.
void ShooterTuningCommand::End(bool interrupted) {
  m_shooter->Stop();
}
