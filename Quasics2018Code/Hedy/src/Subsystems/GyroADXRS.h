#ifndef GYROADXRS_H
#define GYROADXRS_H

#include <Commands/Subsystem.h>
#include <ADXRS450_Gyro.h>

class GyroADXRS: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<frc::ADXRS450_Gyro> analogGyro1;

public:
	GyroADXRS();
	void InitDefaultCommand() override;
	void Periodic() override;

	void Reset();
	void Calibrate();

	double GetAngle();
	double GetRate();
};

#endif
