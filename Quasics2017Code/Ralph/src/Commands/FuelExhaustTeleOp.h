#ifndef FuelExhaustTeleOp_H
#define FuelExhaustTeleOp_H

#include <WPILib.h>

class FuelExhaustTeleOp : public Command {
public:
	FuelExhaustTeleOp(double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
	bool actuatorOpen;
	bool buttonDown;
};

#endif  // FuelExhaustTeleOp_H
