/*
 * DriveBase.h
 *
 *  Created on: Dec 1, 2018
 *      Author: healym
 */

#ifndef SRC_SUBSYSTEMS_DRIVEBASE_H_
#define SRC_SUBSYSTEMS_DRIVEBASE_H_

#include <WPILib.h>

class DriveBase: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<frc::SpeedController> leftFrontMotor;
	std::shared_ptr<frc::SpeedController> leftRearMotor;
	std::shared_ptr<frc::SpeedControllerGroup> leftMotors;
	std::shared_ptr<frc::SpeedController> rightFrontMotor;
	std::shared_ptr<frc::SpeedController> rightRearMotor;
	std::shared_ptr<frc::SpeedControllerGroup> rightMotors;

public:
	DriveBase();
	void InitDefaultCommand();

	void SetPowerToMotors(double leftPercent, double rightPercent);
	void Stop();
};

#endif /* SRC_SUBSYSTEMS_DRIVEBASE_H_ */
