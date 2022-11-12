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

#ifdef READY_FOR_BETA_3
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

// Called when the command is initially scheduled.
void ConveyorTuningCommand::Initialize() {
  m_conveyor->SetConveyorSpeed(m_conveyorSpeedSlider.GetDouble(0));
}

// Called repeatedly when this Command is scheduled to run
void ConveyorTuningCommand::Execute() {
  m_conveyor->SetConveyorSpeed(m_conveyorSpeedSlider.GetDouble(0));
}

// Called once the command ends or is interrupted.
void ConveyorTuningCommand::End(bool interrupted) {
  m_conveyor->Stop();
}
