#ifndef GearServo_H
#define GearServo_H

#include "../Robot.h"

class GearServo : public Command {
public:
	GearServo();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double setAngle();
	double setPower();



};

#endif  // GearServo_H
