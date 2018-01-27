


#ifndef LIMIT_INTAKE_H
#define LIMIT_INTAKE_H


#include "Commands/Subsystem.h"
#include "../Robot.h"



class LimitIntake: public frc::Command {
public:

	LimitIntake();
	std::string gameData;

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:

};

#endif
