#ifndef SWING_TURN_LEFT_H
#define SWING_TURN_LEFT_H

#include <Commands/Command.h>

/**
 *
 *
 * @author ExampleAuthor
 */
// FINDME(mjh): This should seriously be rewritten to derive from frc::TimedCommand,
// instead of rolling our own timer in the code for the class (which is unfortunately
// incorrect in multiple ways...).
class SwingTurnLeft: public frc::Command {
public:
	SwingTurnLeft(double seconds, double power);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	int count;
	double m_seconds;
	double m_power;
};

#endif
