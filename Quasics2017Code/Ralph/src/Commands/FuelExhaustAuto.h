#ifndef FuelExhaustAuto_H
#define FuelExhaustAuto_H

#include <WPILib.h>

class FuelExhaustAuto : public Command {
public:
	FuelExhaustAuto(bool doorOpen);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool openDoor;
};

#endif  // FuelExhaustAuto_H
