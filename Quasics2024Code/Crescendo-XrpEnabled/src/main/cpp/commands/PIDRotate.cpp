// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PIDRotate.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>
#include <frc2/command/PIDCommand.h>
#include <units/math.h>

#include <iostream>

#include "Constants.h"

PIDRotate::PIDRotate(IDrivebase &drivebase, units::degree_t angle)
    : m_drivebase(drivebase),
      m_targetAngle(angle),
      m_pid{PIDTurningConstants::kP, PIDTurningConstants::kI,
            PIDTurningConstants::kD} {
  AddRequirements(&drivebase);

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD

  kP_entry =
      frc::Shuffleboard::GetTab("Tuning Rotation")
          .Add("Turn kP", 0.05)
          .WithWidget(
              frc::BuiltInWidgets::kNumberSlider)  // specify the widget here
          .WithProperties({// specify widget properties here
                           {"min", nt::Value::MakeDouble(0)},
                           {"max", nt::Value::MakeDouble(.1)}})
          .GetEntry();

  kI_entry =
      frc::Shuffleboard::GetTab("Tuning Rotation")
          .Add("Turn kI", 0)
          .WithWidget(
              frc::BuiltInWidgets::kNumberSlider)  // specify the widget here
          .WithProperties({// specify widget properties here
                           {"min", nt::Value::MakeDouble(0)},
                           {"max", nt::Value::MakeDouble(.01)}})
          .GetEntry();
  kD_entry =
      frc::Shuffleboard::GetTab("Tuning Rotation")
          .Add("Turn kD", 0.005)
          .WithWidget(
              frc::BuiltInWidgets::kNumberSlider)  // specify the widget here
          .WithProperties({// specify widget properties here
                           {"min", nt::Value::MakeDouble(0)},
                           {"max", nt::Value::MakeDouble(.01)}})
          .GetEntry();
  angle_entry =
      frc::Shuffleboard::GetTab("Angle Measure")
          .Add("Angle measure", 0.0)
          .WithWidget(
              frc::BuiltInWidgets::kNumberSlider)  // specify the widget here
          .WithProperties({// specify widget properties here
                           {"min", nt::Value::MakeDouble(-180)},
                           {"max", nt::Value::MakeDouble(180)}})
          .GetEntry();
#endif
}

// Called when the command is initially scheduled.
void PIDRotate::Initialize() {
  m_activatePID = false;
  m_currentAngle = 0_deg;
  m_rotationCorrection = 0_deg_per_s;
  m_speed = 30_deg_per_s;
#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  double p = kP_entry->GetDouble(0.0);
  double i = kI_entry->GetDouble(0.0);
  double d = kD_entry->GetDouble(0.0);
  m_targetAngle = units::degree_t(angle_entry->GetDouble(0.0));
  std::cerr << "Using PID values: (" << p << "," << i << "," << d << ")\n";
  std::cerr << "Turning Angle " << m_targetAngle.value() << std::endl;
  m_pid = frc::PIDController(p, i, d);
#endif
  m_pid.Reset();
  m_pid.SetTolerance(ANGLE_TOLERANCE, VELOCITY_TOLERANCE);
  m_pid.EnableContinuousInput(-180, 180);
  m_currentAngle = m_drivebase.getPose().Rotation().Degrees();
}

// Called repeatedly when this Command is scheduled to run
void PIDRotate::Execute() {
  m_currentAngle = m_drivebase.getPose().Rotation().Degrees();
  std::cout << "m_targetAngle: " << m_targetAngle.value()
            << ", m_currentAngle: " << m_currentAngle.value() << std::endl;

  m_rotationCorrection =
      PIDTurningConstants::PID_multiplier *
      (m_pid.Calculate(m_currentAngle.value(), m_targetAngle.value()));
  std::cout << "Sending PID correction Power: " << m_rotationCorrection.value()
            << "Angle Away"
            << std::abs((m_targetAngle - m_currentAngle).value()) << std::endl;

  if (m_rotationCorrection >= 0_deg_per_s) {
    m_drivebase.arcadeDrive(0_mps, m_rotationCorrection + 50_deg_per_s);
  } else {
    m_drivebase.arcadeDrive(0_mps, m_rotationCorrection - 50_deg_per_s);
  }
}

// Called once the command ends or is interrupted.
void PIDRotate::End(bool interrupted) {
  m_drivebase.tankDrive(0, 0);
}

// Returns true when the command should end.
bool PIDRotate::IsFinished() {
  // if the current condition and the correction speed is less than 0.2

  if (units::math::abs(m_targetAngle - m_currentAngle) < 1_deg &&
      units::math::abs(m_drivebase.getWheelSpeeds().left) < 0.3_mps &&
      units::math::abs(m_drivebase.getWheelSpeeds().right) < 0.3_mps) {
    std::cout << "requested angle: " << m_targetAngle.value()
              << "m_currentAngle" << m_currentAngle.value() << std::endl;
    return true;
  }
  return false;
}