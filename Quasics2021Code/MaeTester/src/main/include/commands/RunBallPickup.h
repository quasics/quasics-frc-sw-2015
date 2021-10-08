// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

/**
 * Runs the ball pickup at a specified speed, for a given duration.
 */
class RunBallPickup
    : public frc2::CommandHelper<frc2::CommandBase, RunBallPickup> {
 public:
  /**
   * Constructor.
   *
   * @param intake   pointer to the ball intake mechanism
   * @param power    % power (speed) to run at; runs from -1.0 to +1.0
   * @param duration length of time to run the mechanism
   *
   * @see Intake#SetBallPickupSpeed
   */
  RunBallPickup(Intake* intake, double power, units::second_t duration);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Intake* m_intake;
  double m_power;
  units::second_t m_duration;
  frc2::Timer m_timer;
};
