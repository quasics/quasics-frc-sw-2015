// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunConveyorUntilBallLoads
    : public frc2::CommandHelper<frc2::CommandBase, RunConveyorUntilBallLoads> {
 public:
  RunConveyorUntilBallLoads(Intake* intake, double conveyorPower,
                            double pickupPower,
                            units::second_t timeout = units::second_t(3));

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  enum State {
    eStartingBallSensed,
    eNoBallSensed,
    eNewBallSensed,
    eFinished,
  };

  // Data members
 private:
  Intake* m_intake;
  double m_conveyorPower;
  double m_pickupPower;
  units::second_t m_timeout;
  frc::Timer m_timer;
  State m_state = eNoBallSensed;
};
