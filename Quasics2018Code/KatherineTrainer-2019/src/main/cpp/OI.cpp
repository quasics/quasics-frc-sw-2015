#include "OI.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Commands/AutonomousCommand.h"
#include "Commands/SimpleMoveSequence.h"
#include "Commands/TimedMove.h"
#include "Commands/LeftTurn.h"
#include "Commands/TurnToNorth.h"
#include "Commands/Rotate.h"
#include "Commands/MoveForDuration.h"
#include "Commands/ZigZagSequential.h"
OI::OI() {
    // Process operator interface input here.
	// SmartDashboard Buttons
    // frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());
    //    frc::SmartDashboard::PutData("20% (backward) for 1/2 second", new TimedMove(.5, -.20));
    //    frc::SmartDashboard::PutData("20% for 1/2 second", new TimedMove(.5, .20));
    //    frc::SmartDashboard::PutData("20% for 1 second", new TimedMove(1, .2));
    frc::SmartDashboard::PutData("Square", new SimpleMoveSequence);
    frc::SmartDashboard::PutData("Forward", new TimedMove(3.00, 1.00));
    frc::SmartDashboard::PutData("Compass", new TurnToNorth);
    frc::SmartDashboard::PutData("Turning", new Rotate(2));
     frc::SmartDashboard::PutData("Reverse", new TimedMove(3.00, -1.00));
    frc::SmartDashboard::PutData("Move forward", new MoveForDuration(1.00));
    frc::SmartDashboard::PutData("Zig Zag", new ZigZagSequential);
}
