#ifndef GearAuto_H
#define GearAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class GearAuto : public Command {
public:
	GearAuto(bool doorOpen);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	bool openDoor;
	bool kickerDelay;
	unsigned int counter;
	bool isDone;
};

#endif
