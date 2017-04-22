#ifndef TeleopCommandGroup_H
#define TeleopCommandGroup_H

#include "WPILib.h"
#include <Commands/BaseCommands/Teleop/ClimberTeleop.h>
#include <Commands/BaseCommands/Teleop/GearDoorTeleop.h>
#include <Commands/BaseCommands/Teleop/TankDrive.h>

class TeleopCommandGroup : public CommandGroup {
public:
	TeleopCommandGroup();
};

#endif  // TeleopCommandGroup_H
