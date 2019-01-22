#ifndef SHOULDER_MOVE_H
#define SHOULDER_MOVE_H

#include <frc/commands/Command.h>

/**
 *
 *
 * @author ExampleAuthor
 */
// FINDME(mjh): This should seriously be rewritten to derive from frc::TimedCommand,
// instead of rolling our own timer in the code for the class (which is unfortunately
// incorrect in multiple ways...).
class ShoulderMove: public frc::Command {
public:
	ShoulderMove(double seconds, double power);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	int count;
	double m_power;
	double m_seconds;
};

#endif
