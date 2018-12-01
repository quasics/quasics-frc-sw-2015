#include "OI.h"

#include <SmartDashboard/SmartDashboard.h>

#include "Commands/AutonomousCommand.h"
#include "Commands/TimedMove.h"

OI::OI() {
    // Process operator interface input here.

	// SmartDashboard Buttons
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    frc::SmartDashboard::PutData("50% for 1 second", new TimedMove(1, .50));
    frc::SmartDashboard::PutData("100% for 1 second", new TimedMove(1, 1));
    frc::SmartDashboard::PutData("50% (backward) for 1 second", new TimedMove(1, -.50));
}
