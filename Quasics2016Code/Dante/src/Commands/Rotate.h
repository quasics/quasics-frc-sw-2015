/*
 * Rotate.h
 *
 *  Created on: Nov 3, 2016
 *      Author: axf105
 */

#ifndef SRC_COMMANDS_ROTATE_H_
#define SRC_COMMANDS_ROTATE_H_

#include <Commands/Command.h>

class Rotate: public Command {

private:
	double m_seconds;
	int counter;
	double m_power;
public:
	Rotate(double seconds, double power);
	virtual ~Rotate();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupt();
};

#endif /* SRC_COMMANDS_ROTATE_H_ */
