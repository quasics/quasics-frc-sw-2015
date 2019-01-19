/*
 * LightingDefault.h
 *
 *  Created on: Mar 8, 2018
 *      Author: sth101
 */


#include <frc/commands/Command.h>

class LightingDefault: public frc::Command {
public:
	LightingDefault();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int autoCounter = -1;
	int teleOpCounter = -1;
};

