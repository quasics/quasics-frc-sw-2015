#ifndef FuelExhaustAuto_H
#define FuelExhaustAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class FuelExhaustAuto : public Command {
public:
	FuelExhaustAuto(bool doorOpen);
	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	bool openDoor;
};

#endif  // FuelExhaustAuto_H
