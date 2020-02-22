/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveBase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, TankDriveCommand> {
 public:
  /**
   * Constructor.
   *
   * Note that the functions to supply left and right power are assumed to be
   * fully self-contained, and includes "dead zone" support adjustments (as
   * needed).
   *
   * @param driveBase pointer to the drive base subsystem
   * @param leftPower function returning the power setting for the left side
   * @param rightPower function returning the power setting for the right side
   */
  TankDriveCommand(DriveBase* driveBase, std::function<double()> leftPower,
                   std::function<double()> rightPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  DriveBase* driveBase;
  std::function<double()> m_leftPower;
  std::function<double()> m_rightPower;
};
