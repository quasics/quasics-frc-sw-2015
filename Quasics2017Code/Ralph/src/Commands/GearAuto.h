#ifndef GearAuto_H
#define GearAuto_H

#include <WPILib.h>

class GearAuto : public Command {
public:
	GearAuto(bool doorOpen);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool openDoor;
};

#endif
