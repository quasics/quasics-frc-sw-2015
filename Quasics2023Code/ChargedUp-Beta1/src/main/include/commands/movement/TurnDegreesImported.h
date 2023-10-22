
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
// CODE_REVIEW(matthew): This class should have a comment block (above)
// describing what it is/does, rather than saying that it's an "example
// command".  And how does this differ from the "RotateAtAngle" command?  When
// would we use one vs. the other?
class TurnDegreesImported
    : public frc2::CommandHelper<frc2::Command, TurnDegreesImported> {
 public:
  TurnDegreesImported(Drivebase* drivebase, double speed,
                      units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  /** Drive base used to move the robot. */
  Drivebase* const m_drivebase;
  /** Desired (maximum) speed for the turn. */
  const double m_speed;
  /** Desired amount by which the robot should turn when the command runs. */
  const units::degree_t m_angle;

  /**
   * Current angle (yaw) of the robot when the command starts running.  This
   * allows us to determine when the full turn from the starting position has
   * been completed.
   */
  units::degree_t m_startingposition;

  // CODE_REVIEW(matthew): What is this for?  Why isn't it just a local variable
  // in Execute()?
  double m_subtraction = 0;
};
