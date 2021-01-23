/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoModeBallDelivery.h"

#include "commands/DeliverForTimeCommand.h"
#include "commands/MoveForTimeCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AutoModeBallDelivery::AutoModeBallDelivery(Exhaust* exhaust,
                                           Drivebase* drivebase) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
      // Move to the wall
      MoveForTimeCommand(drivebase, 3, -.3),
      // Deliver Balls
      DeliverForTimeCommand(exhaust, 8),
      // Back up again
      MoveForTimeCommand(drivebase, 3.5, .3));
}
