#ifndef AuxiliaryCommands_H
#define AuxiliaryCommands_H

#include <Commands/CommandGroup.h>
#include "ContinousIntake.h"
#include <Commands/FuelExhaustTeleop.h>
#include <Commands/GearTeleop.h>
#include <Commands/OutputTele.h>

class AuxiliaryCommands : public CommandGroup {
public:
	AuxiliaryCommands();
};

#endif  // AuxiliaryCommands_H
