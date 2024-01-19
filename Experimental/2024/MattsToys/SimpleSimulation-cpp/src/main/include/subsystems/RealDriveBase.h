// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "subsystems/AbstractDriveBase.h"

class RealDriveBase : public AbstractDriveBase {
 public:
  RealDriveBase();

  // Hardware abstraction layer
 protected:
  void setMotorSpeeds(double leftPercent, double rightPercent) override;

  // Data members (actual motor control)
 private:
  rev::CANSparkMax m_leftFront{MotorIds::SparkMax::LEFT_FRONT_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFront{MotorIds::SparkMax::RIGHT_FRONT_DRIVE_MOTOR_ID,
                                rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftBack{MotorIds::SparkMax::LEFT_BACK_DRIVE_MOTOR_ID,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightBack{MotorIds::SparkMax::RIGHT_BACK_DRIVE_MOTOR_ID,
                               rev::CANSparkMax::MotorType::kBrushless};

#if !defined(ENABLE_MOTOR_CONTROLLER_GROUPS) && defined(BACK_MOTORS_ARE_LEADERS)
  frc::MotorController& m_leftSide = m_leftBack;
  frc::MotorController& m_rightSide = m_rightBack;
#elif !defined(ENABLE_MOTOR_CONTROLLER_GROUPS) && \
    !defined(BACK_MOTORS_ARE_LEADERS)
  frc::MotorController& m_leftSide = m_leftFront;
  frc::MotorController& m_rightSide = m_rightFront;
#else
  // Motor controller groups, pairing sets on left/right.
  frc::MotorControllerGroup m_leftSide{m_leftFront, m_leftBack};
  frc::MotorControllerGroup m_rightSide{m_rightFront, m_rightBack};
#endif
};
