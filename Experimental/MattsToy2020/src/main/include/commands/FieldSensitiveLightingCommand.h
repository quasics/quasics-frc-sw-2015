/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/LightingSubsystem.h"

/**
 * A sample implementation of controls for configurable LED strips on the robot
 * that can use data from the Field Management System (FMS) to set them to the
 * color of our current alliance (or some preferred default).
 */
class FieldSensitiveLightingCommand
    : public frc2::CommandHelper<frc2::CommandBase,
                                 FieldSensitiveLightingCommand> {
 public:
  /**
   * Constructor.
   *
   * @param lighting   the lighting subystem with which we interact
   */
  FieldSensitiveLightingCommand(LightingSubsystem* lighting)
      : lighting(lighting) {
    AddRequirements(lighting);
  }

  /**
   * Updates the lighting value (in case data from the FMS has changed).
   */
  void Execute() override;

  /**
   * Sets the lights back to a default color (green).
   */
  void End(bool interrupted) override;

 private:
  LightingSubsystem* lighting;  ///< The lighting subsystem.
};
