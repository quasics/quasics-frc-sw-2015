// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#undef INTAKE_USES_LIMIT_SWITCH
#define INTAKE_USES_BEAM_SENSOR

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Controls what the commmands that intake (or exhaust) the balls.
  void IntakeBallOn();
  void IntakeBallReverse();
  void IntakeBallOff();

  void ConveyBallOn();
  void ConveyBallReverse();
  void ConveyBallOff();

  void OnlyIntakeOn();
  void OnlyIntakeReverse();
  void OnlyIntakeOff();
  void IntakeCellsAuto();

  bool IsBallInChamber();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX intakeMotor;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX conveyorMotor;

#if defined(INTAKE_USES_LIMIT_SWITCH)
  std::shared_ptr<frc::DigitalInput> conveyorLimitSwitch;
#elif defined(INTAKE_USES_BEAM_SENSOR)
  std::shared_ptr<frc::DigitalInput> conveyorBeamSensor;
#endif
};
