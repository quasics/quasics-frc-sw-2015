#include "AutoLeftSwitch.h"
#include "Primitives/MoveForDistance.h"
#include "Primitives/Outtake.h"
#include "GyroTurn.h"


AutoLeftSwitch::AutoLeftSwitch() {
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

	AddSequential(new MoveForDistance(144, -.3));		// Move forward, to the switch.

	AddSequential(new GyroTurn(90, .3));               //Turn 90 degrees right to face the switch

	AddSequential(new MoveForDistance(18, -.3));		// Move forward, to the switch.

	//AddSequential(new Outtake(2, -.7));		        // Spit cube into switch.

}
