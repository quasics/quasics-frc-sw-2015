#ifndef ActuatorAuto_H
#define ActuatorAuto_H

#include <WPILib.h>

class ActuatorAuto : public Command {
public:
	ActuatorAuto(bool doorOpen);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();
	void Execute() override;

private:
	const bool openDoor;

};

#endif  // ActuatorAuto_H
