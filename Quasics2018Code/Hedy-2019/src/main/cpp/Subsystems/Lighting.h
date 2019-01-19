// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef LIGHTING_H
#define LIGHTING_H

#include <frc/WPILib.h>
#include <frc/SerialPort.h>

/**
 *
 *
 * @author ExampleAuthor
 */
class Lighting: public frc::Subsystem {
private:
	std::shared_ptr<frc::SerialPort> serialPort;

	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURC E=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private:
	void SendCommand(const char* str);
public:
	Lighting();
	void InitDefaultCommand() override;
	void Periodic() override;
	void SendTestString();
	void WriteOn();
	void WriteOff();
	void WriteBlue();
	void WriteGreen();
	void WriteRed();
	void WriteAuto();
	void WriteTeleOp();
	char GetColor();
	char GetMode();
	void Stop();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};

#endif
