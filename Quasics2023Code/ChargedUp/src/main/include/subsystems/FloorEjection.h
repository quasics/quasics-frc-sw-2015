// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

/**
 * TODO: Add comments describing the class as a whole.
 */
// CODE_REVIEW(matthew): This class should have a comment block (above)
// describing what it is/does.  (And the functions should probably be
// documented, too, since some of them aren't necessarily intuitively obvious.)
//
// CODE_REVIEW(matthew): Given that this hardware is no longer installed in
// the robot, this subsystem (and commands that use it) should probably be
// removed from the code base.  (This caused a problem during match 4 at the
// Steel City Showdown event, since the selected auto mode command used this,
// and thus the robot failed to work correctly in that match.)
class FloorEjection : public frc2::SubsystemBase {
 public:
  FloorEjection();

  void SetFloorEjectionPower(double power);

  double GetPosition();

  void ResetEncoder();

  double GetVelocity();

  void Stop();

  bool FloorRetracted();

  void SetBrakingMode(bool brake);

  // Functions common to all subsystems.
 public:
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::VictorSPX m_floorEjectionMotor{
      MotorIds::VictorSPX::GAME_PIECE_FLIPPER_ID};

  frc::DutyCycleEncoder m_floorEjectionEncoder{
      DigitalInput::FLOOR_INTAKE_ENCODER};

  frc::DigitalInput m_floorRetractionLimitSwitch{
      DigitalInput::FLOOR_RETRACTION_LIMIT_SWITCH_ID};

  // frc::Encoder m_floorEjectionEncoder{DigitalInput::FLOOR_INTAKE_ENCODER};
#ifdef ENABLE_FLOOR_EJECTION_ENCODER

  frc::Encoder m_floorEjectionEncoder{ThroughBore::A_CHANNEL,
                                      ThroughBore::B_CHANNEL};
#endif
};
