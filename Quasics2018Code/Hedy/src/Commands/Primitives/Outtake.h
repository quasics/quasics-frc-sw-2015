#ifndef OUTTAKE_H
#define OUTTAKE_H

#include <Commands/Command.h>

/**
 * Used during autonomous mode to expel the cube.
 *
 * @author ExampleAuthor
 */
class Outtake: public frc::Command {
public:
	Outtake(double seconds, double power);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	int count;
	double m_power;
	double m_ticksToRun;
};

#endif
