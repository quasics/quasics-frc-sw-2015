#ifndef FuelAutoRed_H
#define FuelAutoRed_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
// CODE_REVIEW(mjh): What distinguishes this from the "FuelAuto" command,
// and why can't they be a single class that (perhaps) takes a parameter
// on the constructor to differentiate them?
//For Red Alliance
class FuelAutoRed : public CommandGroup {
public:
	FuelAutoRed();
};

#endif  // FuelAutoRed_H
