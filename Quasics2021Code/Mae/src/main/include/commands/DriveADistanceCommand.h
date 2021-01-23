/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivebase.h"

/// @todo (Scott) Document this class (using JavaDoc format).
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveADistanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveADistanceCommand> {
 public:
  /**
   * @param distance   distince (in inches) to move, as reported by *either* the
   *                   left or right encoders (i.e., once either side reports
   *                   this distance, we'll stop)
   */
  DriveADistanceCommand(Drivebase* drivebase, double distance, double power);

  /**
   * @param distance   distince (in inches) to move, as reported by *either* the
   *                   left or right encoders (i.e., once either side reports
   *                   this distance, we'll stop)
   */
  DriveADistanceCommand(Drivebase* drivebase, double distance, double leftPower,
                        double rightPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drivebase* drivebase;
  const double distance;
  const double leftPower;
  const double rightPower;
};
