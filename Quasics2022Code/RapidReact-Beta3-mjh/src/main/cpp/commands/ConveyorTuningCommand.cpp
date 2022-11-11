// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ConveyorTuningCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>

#include "Constants.h"

ConveyorTuningCommand::ConveyorTuningCommand(Conveyor* conveyor,
                                             double initialConveyorSpeedPercent)
    : m_conveyor(conveyor) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_conveyor);

#if 0 // BROKEN_IN_BETA_2
  // The following code is broken when built under Beta 2.
  // 
  // Bug filed: https://github.com/wpilibsuite/BetaTest/issues/80
  wpi::StringMap<std::shared_ptr<nt::Value>> speedSliderProperties{
      {"min", nt::Value::MakeDouble(-1.0)},
      {"max", nt::Value::MakeDouble(+1.0)},
      {"Block increment", nt::Value::MakeDouble(0.01)}};

  auto& tab = frc::Shuffleboard::GetTab(NetworkTableNames::kSettingsTab);
  m_conveyorSpeedSlider =
      tab.AddPersistent(NetworkTableNames::kConveyorSpeedSliderName,
                        initialConveyorSpeedPercent)
          .WithWidget(frc::BuiltInWidgets::kNumberSlider)
          .WithProperties(speedSliderProperties)
          .GetEntry();
#endif
}

void ConveyorTuningCommand::UpdateSpeed() {
  if (m_conveyorSpeedSlider) {
    m_conveyor->SetConveyorSpeed(m_conveyorSpeedSlider.GetDouble(0));
  }
}

// Called when the command is initially scheduled.
void ConveyorTuningCommand::Initialize() {
  UpdateSpeed();
}

// Called repeatedly when this Command is scheduled to run
void ConveyorTuningCommand::Execute() {
  UpdateSpeed();
}

// Called once the command ends or is interrupted.
void ConveyorTuningCommand::End(bool interrupted) {
  m_conveyor->Stop();
}
