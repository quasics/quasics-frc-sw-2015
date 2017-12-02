#ifndef SRC_COMMANDS_TURNLEFTINPUT_H_
#define SRC_COMMANDS_TURNLEFTINPUT_H_

#include <Commands/Command.h>

class TurnRightInput : public frc::Command {
public:
	TurnRightInput(double degrees);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	const double DegreesToTurn;
	double Goal;
};
#endif /* SRC_COMMANDS_TURNLEFTINPUT_H_ */
