#ifndef GearAuto_H
#define GearAuto_H

#include <WPILib.h>

class GearAuto : public Command {
public:
	GearAuto(bool doorOpen);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool openDoor;
	bool kickerDelay;
	unsigned int counter;
	bool isDone;
};

#endif
