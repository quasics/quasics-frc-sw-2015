#ifndef GearTeleop_H
#define GearTeleop_H

#include <WPILib.h>

// CODE_REVIEW(mjh): This #include should be moved into the .cpp file.  (Build-time
// improvements.)
#include "../Subsystems/Gear.h"

// CODE_REVIEW(mjh): Document what this class actually does.
class GearTeleop : public Command {
public:
	GearTeleop();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End ();
	void Interrupted();

private:
	bool previousValue;
	uint32_t counter;
	bool isOpening;
};

#endif  // GearTeleop_H
