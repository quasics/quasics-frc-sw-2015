#ifndef SRC_COMMANDS_MOVEFORWARDONEMETER_H_
#define SRC_COMMANDS_MOVEFORWARDONEMETER_H_

#include <Commands/Command.h>

class MoveForwardOneMeter : public frc::Command {
public:
	MoveForwardOneMeter();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
};
#endif /* SRC_COMMANDS_MOVEFORWARDONEMETER_H_ */
