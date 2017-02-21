#ifndef AuxiliaryCommands_H
#define AuxiliaryCommands_H

#include <Commands/CommandGroup.h>
// CODE_REVIEW(mjh): All of these should be moved into the .cpp file, since
// the underlying types the define aren't used here.  (Including them here
// will increase build time, since everyone that pulls in this header will
// then also pull *them* in, even though they aren't used by client code.)
#include "IntakeTeleop.h"
#include "FuelExhaustTeleop.h"
#include "GearTeleop.h"
#include "OutputTele.h"

class AuxiliaryCommands : public CommandGroup {
public:
	AuxiliaryCommands();
};

#endif  // AuxiliaryCommands_H
