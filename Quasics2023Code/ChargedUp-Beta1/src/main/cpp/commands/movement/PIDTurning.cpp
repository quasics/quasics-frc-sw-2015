// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/PIDTurning.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>

#include <iostream>

PIDTurning::PIDTurning(Drivebase* drivebase, units::degree_t angle)
    : m_drivebase(drivebase), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);

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
void PIDTurning::Initialize() {
  m_feedForward = true;
  m_activatePID = false;
  m_startingAngle = m_drivebase->GetYaw();
  m_currentAngle = 0_deg;  // Overwritten in FeedForward()
  m_feedForward = true;
  m_activatePID = false;
  m_rotationCorrection = 0;
  // CODE_REVIEW(matthew): Avoid "magic numbers".  You should indicate where
  // this value is coming from (experimental/tuning data, or guessing, etc.), so
  // that it can be updated if needed.
  m_speed = 0.5;
  m_subtraction = 0;

#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  double p = kP_entry->GetDouble(0.0);
  double i = kI_entry->GetDouble(0.0);
  double d = kD_entry->GetDouble(0.0);
  angle = angle_entry->GetDouble(0.0);
  std::cerr << "Using PID values: (" << p << "," << i << "," << d << ")\n";
  std::cerr << "Turning Angle " << angle;
  dynamicPid.reset(new frc::PIDController(p, i, d));
#else
  m_pid.Reset();
  m_pid.SetTolerance(ANGLE_TOLERANCE, VELOCITY_TOLERANCE);
#endif
  FeedForward();
}

// Called repeatedly when this Command is scheduled to run
void PIDTurning::Execute() {
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
  m_drivebase->ArcadeDrive(0, m_rotationCorrection);

#else

  FeedForward();
  // NEEDS TO BE TESTED

  if (m_angle >= 0_deg) {
    if (((m_startingAngle + m_angle - m_currentAngle).value() < 1) &&
        m_feedForward == true) {
      std::cout << "Turning off the feedforward" << std::endl;
      m_feedForward = false;
    }
  } else {
    if (((m_startingAngle + m_angle - m_currentAngle).value() > -1) &&
        m_feedForward == true) {
      std::cout << "Turning off the feedforward" << std::endl;
      m_feedForward = false;
    }
  }
  /*
    if ((std::abs((startingAngle + m_angle - m_currentAngle).value()) < 1) &&
        m_feedForward == true) {
      // std::cout << "Turning off the feedforward" << std::endl;
      m_feedForward = false;
    }*/
  if (!m_feedForward) {
    m_rotationCorrection = m_pid.Calculate(
        m_currentAngle.value(), m_startingAngle.value() + m_angle.value());
    std::cout << "Sending PID correction Power: " << m_rotationCorrection
              << "Angle Away"
              << std::abs((m_startingAngle + m_angle - m_currentAngle).value())
              << std::endl;

    /*
    if (m_rotationCorrection >= 0) {
      m_drivebase->ArcadeDrive(0, m_rotationCorrection + 0.2);
    } else {
      m_drivebase->ArcadeDrive(0, m_rotationCorrection - 0.2);
    }


    */
    if (m_rotationCorrection >= 0) {
      m_drivebase->ArcadeDrive(0, m_rotationCorrection + 0.15);
    } else {
      m_drivebase->ArcadeDrive(0, m_rotationCorrection - 0.15);
    }
  }

#endif
}

// Called once the command ends or is interrupted.
void PIDTurning::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool PIDTurning::IsFinished() {
#ifdef USE_DYNAMIC_DATA_FROM_DASHBOARD
  if (std::abs((m_startingAngle.value() + angle - m_currentAngle.value())) <
      1) {
    return true;
  }
  return false;
#else
  // if the current condition and the correction speed is less than 0.2
  if ((std::abs((m_startingAngle + m_angle - m_currentAngle).value()) < 1) &&
      ((std::abs(m_rotationCorrection) < 0.3) &&
       (std::abs((m_drivebase->GetLeftVelocity()).value()) < 0.05 &&
        std::abs(m_drivebase->GetRightVelocity().value()) < 0.05))) {
    std::cout << "Finished: StartingValue: " << m_startingAngle.value()
              << "requested angle: " << m_angle.value() << "m_currentAngle"
              << m_currentAngle.value() << std::endl;
    return true;
  }
  return false;
#endif
}

void PIDTurning::FeedForward() {
  m_currentAngle = m_drivebase->GetYaw();
  /*if (m_feedForward) {
    if (m_angle > 0_deg) {
      std::cout << "sending power for turning left" << std::endl;
      m_drivebase->ArcadeDrive(0, 0.5);
    } else {
      std::cout << "sending power for turning right" << std::endl;
      m_drivebase->ArcadeDrive(0, -0.5);
    }
  }
*/
  if (m_feedForward) {
    if (std::abs((m_startingAngle + m_angle - m_currentAngle).value()) < 45 &&
        (std::abs(m_speed) > 0.3)) {
      m_subtraction = std::abs(m_speed) - 0.3;
      std::cout << "Invoking subtraction." << std::endl;
    }
    m_drivebase->SetBrakingMode(true);
    if (m_angle > 0_deg) {
      std::cout << "New Speed: " << (m_speed - m_subtraction) << std::endl;
      m_drivebase->ArcadeDrive(0, (m_speed - m_subtraction));
    } else {
      std::cout << "New Speed: " << (m_speed + m_subtraction) << std::endl;
      m_drivebase->ArcadeDrive(0, (-m_speed + m_subtraction));
    }
  }
}