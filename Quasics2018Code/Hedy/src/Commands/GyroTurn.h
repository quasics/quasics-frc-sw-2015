#ifndef GYROTURN_H
#define GYROTURN_H

#include <Commands/Command.h>
#include "Subsystems/GyroADXRS.h"

class GyroTurn: public frc::Command {
public:
	GyroTurn(float angle, double power);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:

	double m_power;
	float m_angle;
};

#endif
