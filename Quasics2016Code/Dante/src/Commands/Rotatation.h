/*
 * Rotatation.h
 */

#ifndef SRC_COMMANDS_ROTATATION_H_
#define SRC_COMMANDS_ROTATATION_H_

#include <Commands/Command.h>
#include <Commands/Rotatation.h>


class Rotatation: public Command {

private:
	double m_seconds;
	double m_power;
	int counter;

public:
	Rotatation(double seconds, double power);
	virtual ~Rotatation();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

};

#endif /* SRC_COMMANDS_ROTATATION_H_ */
