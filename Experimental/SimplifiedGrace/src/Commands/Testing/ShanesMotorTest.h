/*
 * ShanesMotorTest.h
 *
 *  Created on: Dec 14, 2017
 *      Author: healym
 */

#ifndef SRC_COMMANDS_TESTING_SHANESMOTORTEST_H_
#define SRC_COMMANDS_TESTING_SHANESMOTORTEST_H_

#include <Commands/Command.h>
#include "../../Robot.h"
#include "../../Subsystems/HardwareTesting.h"

class ShanesMotorTest: public frc::Command {
private:
	const double power;
public:
	ShanesMotorTest(double powerVal) : power(powerVal) {
		Requires(Robot::hardwareTesting.get());
	}
	void Initialize() override {
		Robot::hardwareTesting->setSpareMotor1Power(power);
		Robot::hardwareTesting->setSpareMotor2Power(power);
	}
	void Execute() override {
		// Nothing to do.
	}
	bool IsFinished() override { return false; }
	void End() override {
		Robot::hardwareTesting->setSpareMotor1Power(0);
		Robot::hardwareTesting->setSpareMotor2Power(0);
	}
	void Interrupted() override {
		End();
	}
};

#endif /* SRC_COMMANDS_TESTING_SHANESMOTORTEST_H_ */
