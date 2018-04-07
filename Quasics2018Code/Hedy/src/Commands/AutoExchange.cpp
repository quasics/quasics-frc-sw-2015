/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "AutoExchange.h"
#include "Primitives/MoveForDistance.h"
#include "Primitives/PointTurnLeft.h"
#include "Primitives/PointTurnRight.h"
#include "Primitives/Outtake.h"
#include <iostream>


AutoExchange::AutoExchange() {
	// Add Commands here:
	// e.g. AddSequential(new Command1());
	//      AddSequential(new Command2());
	// these will run in order.

	// To run multiple commands at the same time,
	// use AddParallel()
	// e.g. AddParallel(new Command1());
	//      AddSequential(new Command2());
	// Command1 and Command2 will run in parallel.

	// A command group will require all of the subsystems that each member
	// would require.
	// e.g. if Command1 requires chassis, and Command2 requires arm,
	// a CommandGroup containing them would require both the chassis and the
	// arm.
	AddSequential(new MoveForDistance(120, -.4));		// Move 10' forward, to cross the line.
	AddSequential(new PointTurnLeft(180, -.4));		    // Turn 180 degrees.
	AddSequential(new MoveForDistance(60, -.4));		// Move 5' forward.
	AddSequential(new PointTurnLeft(90, -.4));		    // Turn 90 degrees left.
	AddSequential(new MoveForDistance(60, -.4));		// Move 5' forward.
	AddSequential(new PointTurnRight(90, -.4));		    // Turn 90 degrees right. to face the exchange.
	AddSequential(new MoveForDistance(60, -.4));		// Move 5' forward.
	AddSequential(new Outtake(2, -.7));		            // Spit cube into exchange window.
	AddSequential(new MoveForDistance(24, .4));		    // Move 2' reverse.

}
