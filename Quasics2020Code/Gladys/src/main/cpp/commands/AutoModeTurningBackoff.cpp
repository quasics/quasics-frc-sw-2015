/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoModeTurningBackoff.h"

#include "commands/DeliverForTimeCommand.h"
#include "commands/DriveADistanceCommand.h"
#include "commands/PointTurnToAnAngleCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AutoModeTurningBackoff::AutoModeTurningBackoff(Exhaust* exhaust,
                                               Drivebase* drivebase) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
      DriveADistanceCommand(drivebase, 98, .3),
      DeliverForTimeCommand(exhaust, 3),
      DriveADistanceCommand(drivebase, -12, -.3),
      PointTurnToAnAngleCommand(drivebase, false, 40),
      // following drives backward around two feet and sideways around two feet)
      DriveADistanceCommand(drivebase, -50, -.3),
      PointTurnToAnAngleCommand(drivebase, true, 55),
      // Drives the remaining distance to the line
      DriveADistanceCommand(drivebase, -48, -.3));
}
