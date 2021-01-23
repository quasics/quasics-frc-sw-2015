/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climber.h"


/**
 * Move the climber down
 */ 
class ClimberDownCommand
    : public frc2::CommandHelper<frc2::CommandBase, ClimberDownCommand> {
 public:
  ClimberDownCommand(Climber* climber);

/**
 * Move the climber down
 */ 
  void Execute() override; 

/**
 * Stop the climber
 */ 
  void End(bool interrupted) override; 

 private:
  Climber* climber;
};
