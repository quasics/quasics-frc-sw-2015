#include "OI.h"

#include <SmartDashboard/SmartDashboard.h>

#include "Commands/AutonomousCommand.h"
#include "Commands/SimpleMoveSequence.h"
#include "Commands/TimedMove.h"
#include "Commands/LeftTurn.h"
#include "Commands/TurnToNorth.h"
OI::OI() {
    // Process operator interface input here.

	// SmartDashboard Buttons
    // frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

//    frc::SmartDashboard::PutData("20% (backward) for 1/2 second", new TimedMove(.5, -.20));
//    frc::SmartDashboard::PutData("20% for 1/2 second", new TimedMove(.5, .20));
//    frc::SmartDashboard::PutData("20% for 1 second", new TimedMove(1, .2));
    frc::SmartDashboard::PutData("Square", new SimpleMoveSequence);
    frc::SmartDashboard::PutData("Straight", new TimedMove(1.00, .2,.3));
    frc::SmartDashboard::PutData("Compass", new TurnToNorth);
}
