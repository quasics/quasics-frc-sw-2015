#ifndef moveForward_H
#define moveForward_H

#include "../CommandBase.h"
#include "../Robot.h"

class MoveForward : public CommandBase {
public:
	MoveForward(double powerLevel, double seconds);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double power;
	double sec;
	int counter;
};

#endif  // moveForward_H
