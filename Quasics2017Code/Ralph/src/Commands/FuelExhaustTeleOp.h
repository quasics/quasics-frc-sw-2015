#ifndef FuelExhaustTeleOp_H
#define FuelExhaustTeleOp_H

#include <WPILib.h>

class FuelExhaustTeleOp : public Command {
public:
	FuelExhaustTeleOp();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	bool actuatorOpen;
	bool buttonDown;
	bool buttonPrevious;
};

#endif  // FuelExhaustTeleOp_H
