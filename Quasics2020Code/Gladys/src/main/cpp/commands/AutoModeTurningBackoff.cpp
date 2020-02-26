/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoModeTurningBackoff.h"
#include "commands/DriveADistance.h"
#include "commands/PointTurnToAnAngleCommand.h"
#include "commands/DeliverForTime.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
AutoModeTurningBackoff::AutoModeTurningBackoff(Exhaust* exhaust, Drivebase* drivebase){
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(
    DriveADistance(drivebase, 102, .3),
    DeliverForTime(exhaust, 3),
    DriveADistance(drivebase, -6, -.3),
    PointTurnToAnAngleCommand(drivebase, true, 45),
    //following drives backward around two feet and sideways around two feet)
    DriveADistance(drivebase, -.3, 34),
    PointTurnToAnAngleCommand(drivebase, false, 45),
    //Drives the remaining distance to the line
    DriveADistance(drivebase, -92, -.3)
  );
}
