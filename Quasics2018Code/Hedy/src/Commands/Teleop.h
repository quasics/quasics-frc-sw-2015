#ifndef Teleop_H
#define Teleop_H

#include <Commands/CommandGroup.h>
#include "TankDrive.h"
#include "TeleOut.h"
#include "TeleIn.h"

class Teleop : public CommandGroup {
public:
	Teleop();
};

#endif  // Teleop_H
