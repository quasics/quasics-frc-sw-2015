#ifndef FuelExhaustTeleOp_H
#define FuelExhaustTeleOp_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class FuelExhaustTeleOp : public Command {
public:
	FuelExhaustTeleOp();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	bool actuatorOpen;
	bool buttonDown;
	bool buttonPrevious;
};

#endif  // FuelExhaustTeleOp_H
