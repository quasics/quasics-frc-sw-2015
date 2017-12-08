#ifndef SRC_COMMANDS_EXPERIMENTALTWO_H_
#define SRC_COMMANDS_EXPERIMENTALTWO_H_

#include <Commands/Command.h>

class ExperimentalTwo : public frc::Command{
public:
	ExperimentalTwo();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_EXPERIMENTALTWO_H_ */
