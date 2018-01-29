/*
 * HardwareTesting.h
 *
 *  Created on: Dec 14, 2017
 *      Author: healym
 */

#ifndef SRC_SUBSYSTEMS_HARDWARETESTING_H_
#define SRC_SUBSYSTEMS_HARDWARETESTING_H_

#include <WPILib.h>
#include <Commands/Subsystem.h>

class HardwareTesting: public frc::Subsystem {
private:
	std::shared_ptr<SpeedController> spareMotor1;
	std::shared_ptr<SpeedController> spareMotor2;
public:
	HardwareTesting();

	void setSpareMotor1Power(double power);
	void setSpareMotor2Power(double power);
};

#endif /* SRC_SUBSYSTEMS_HARDWARETESTING_H_ */
