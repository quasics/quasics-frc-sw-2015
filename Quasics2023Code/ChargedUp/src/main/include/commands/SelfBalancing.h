// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Uses PID controll to sense the angle of the robot and adjust the movement so
// the robot becomes level
class SelfBalancing
    : public frc2::CommandHelper<frc2::CommandBase, SelfBalancing> {
 public:
  SelfBalancing(Drivebase* drivebase);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  Drivebase* m_drivebase;
  // Used to calculate the power to give to the wheels
  frc2::PIDController pid{SelfBalancingConstants::PID::kP,
                          SelfBalancingConstants::PID::kI,
                          SelfBalancingConstants::PID::kD};
  // Saves the angle that the robot was on for subsequent slope
  // identification(see slopeOfRamp)
  double pastAngle;
  // flag that turns off the FeedForward command(a.k.a balancing before PID
  // Controller)
  bool noFeedFowardPower = false;
  // flag that tells the robot when to begin finetuning the balancing with PID
  // controller
  bool activatePID = false;
  // Coefficient to change direction of power given to motors based of pastAngle
  // pastAngle > 0 positive
  // pastAngle < 0 negative
  double slopeOfRamp = 1;
};
