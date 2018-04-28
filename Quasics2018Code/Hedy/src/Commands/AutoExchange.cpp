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
#include "GyroTurn.h"
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

	//Power levels should be ~30% for forward moves, and ~15% for turns on tile. ~30% for turns on carpet.

	AddSequential(new MoveForDistance(120, -.3));		// Move forward, to cross the line.

	//Either the triangle or the box can be used, depending on placement.

	//triangular moving

	AddSequential(new GyroTurn(135, .3));               //Turn 135 degrees.

	AddSequential(new MoveForDistance(60, -.3));        //Move forward.

	AddSequential(new GyroTurn(30, .3));                //Turn 20 degrees.

	AddSequential(new MoveForDistance(60, -.3));        //Move to the exchange.

	//end triangle code
/*
    //box moving
     *
	AddSequential(new GyroTurn(180, .3));               //Turn 180 degrees.

	AddSequential(new MoveForDistance(55, -.3));		// Move forward.

	AddSequential(new GyroTurn(-90, -.25));             //Turn 90 degrees left

	AddSequential(new MoveForDistance(50, -.3));		// Move forward.

	AddSequential(new GyroTurn(90, .3));                //Turn 90 degrees right to face the exchange.

	AddSequential(new MoveForDistance(48, -.3));		// Move forward to the exchange.

	//end box moving

*/
	//AddSequential(new Outtake(2, -.7));		            // Spit cube into exchange window.

	AddSequential(new MoveForDistance(36, .4));		    // Move 3' reverse.

	AddSequential(new GyroTurn(160, .3));               //Turn 160 degrees.

}
