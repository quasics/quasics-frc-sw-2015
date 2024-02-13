// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PIDRotate.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>
#include <frc2/command/PIDCommand.h>
#include <units/math.h>

#include <iostream>

PIDRotate::PIDRotate(IDrivebase &drivebase, units::degree_t angle)
    : m_drivebase(drivebase), m_targetAngle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
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
          .Add("Turn kI", 0.005)
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
          .Add("Turn kD", 0.0)
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
  m_feedForward = true;
  m_activatePID = false;
  m_currentAngle = 0_deg;  // Overwritten in FeedForward()
  m_rotationCorrection = 0_deg_per_s;
  m_speed = 30_deg_per_s;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  double p = kP_entry->GetDouble(0.0);
  double i = kI_entry->GetDouble(0.0);
  double d = kD_entry->GetDouble(0.0);
  angle = angle_entry->GetDouble(0.0);
  std::cerr << "Using PID values: (" << p << "," << i << "," << d << ")\n";
  std::cerr << "Turning Angle " << angle;
  dynamicPid.reset(new frc2::PIDController(p, i, d));
#else
  m_pid.Reset();
  m_pid.SetTolerance(ANGLE_TOLERANCE, VELOCITY_TOLERANCE);
#endif
  FeedForward();
}

// Called repeatedly when this Command is scheduled to run
void PIDRotate::Execute() {
#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  FeedForward();
  if (std::abs(m_startingAngle.value() + angle - m_currentAngle.value()) < 10) {
    m_feedForward = false;
  }
  if (!m_feedForward) {
    m_rotationCorrection =
        (dynamicPid == nullptr)
            ? 0.0
            : dynamicPid->Calculate(m_currentAngle.value(),
                                    m_startingAngle.value() + angle);
  }
  m_drivebase.arcadeDrive(0_mps, m_rotationCorrection);

#else

  FeedForward();

  while (m_currentAngle + 180_deg < m_targetAngle) {
    m_targetAngle -= 360_deg;
  }
  while (m_currentAngle - 180_deg > m_targetAngle) {
    m_targetAngle += 360_deg;
  }
  std::cout << "m_targetAngle: " << m_targetAngle.value()
            << ", m_currentAngle: " << m_currentAngle.value() << std::endl;

  // NEEDS TO BE TESTED
  /*
    if (m_angle >= 0_deg) {
      if (((m_angle - m_currentAngle).value() < 1) && m_feedForward == true) {
        std::cout << "Turning off the feedforward" << std::endl;
        m_feedForward = false;
      }
    } else {
      if (((m_angle - m_currentAngle).value() > -1) && m_feedForward == true) {
        std::cout << "Turning off the feedforward" << std::endl;
        m_feedForward = false;
      }
    }*/
  /*
    if ((std::abs((startingAngle + m_angle - m_currentAngle).value()) < 1) &&
        m_feedForward == true) {
      // std::cout << "Turning off the feedforward" << std::endl;
      m_feedForward = false;
    }*/
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

#endif
}

// Called once the command ends or is interrupted.
void PIDRotate::End(bool interrupted) {
  m_drivebase.tankDrive(0, 0);
}

// Returns true when the command should end.
bool PIDRotate::IsFinished() {
#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  if (std::abs((m_startingAngle.value() + angle - m_currentAngle.value())) <
      1) {
    return true;
  }
  return false;
#else
  // if the current condition and the correction speed is less than 0.2

  if (units::math::abs(m_targetAngle - m_currentAngle) < 1_deg &&
      units::math::abs(m_rotationCorrection) < 0.8_deg_per_s &&
      units::math::abs(m_drivebase.getWheelSpeeds().left) < 0.05_mps &&
      units::math::abs(m_drivebase.getWheelSpeeds().right) < 0.05_mps) {
    std::cout << "requested angle: " << m_targetAngle.value()
              << "m_currentAngle" << m_currentAngle.value() << std::endl;
    return true;
  }
  return false;
#endif
}

void PIDRotate::FeedForward() {
  m_currentAngle = m_drivebase.getYaw();

  /*if (m_feedForward) {
    if (m_angle > 0_deg) {
      std::cout << "sending power for turning left" << std::endl;
      m_drivebase.arcadeDrive(0_mps, 0.5);
    } else {
      std::cout << "sending power for turning right" << std::endl;
      m_drivebase.arcadeDrive(0_mps, -0.5);
    }
  }
/*
/
  if (m_angle > 0_deg) {
    std::cout << "New Speed: " << (m_speed) << std::endl;
    m_drivebase.arcadeDrive(0_mps, units::degrees_per_second_t(m_speed));
  } else {
    std::cout << "New Speed: " << (m_speed) << std::endl;
    m_drivebase.arcadeDrive(0_mps, units::degrees_per_second_t(-m_speed));
  }*/
}