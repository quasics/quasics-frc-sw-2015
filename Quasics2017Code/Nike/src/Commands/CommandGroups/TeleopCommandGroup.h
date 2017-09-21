#ifndef TeleopCommandGroup_H
#define TeleopCommandGroup_H

#include <WPILib.h>
#include "../BaseCommands/Teleop/ClimberTeleop.h"
#include "../BaseCommands/Teleop/GearDoorTeleop.h"
#include "../BaseCommands/Teleop/TankDrive.h"

class TeleopCommandGroup : public CommandGroup {
public:
	TeleopCommandGroup();
};

#endif  // TeleopCommandGroup_H
