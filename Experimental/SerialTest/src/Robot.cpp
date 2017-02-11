#include <iostream>
#include <memory>
#include <string>

#include "WPILib.h"

const std::string AutoInitString = "AutoInit\n";
const std::string AutoPeriodicString = "AutoPeriodic\n";
const std::string TeleopInitString = "TeleopInit\n";
const std::string TeleopPeriodicString = "TeleopPeriodic\n";
const std::string TestInitString = "TestInit\n";
const std::string TestPeriodicString = "TestPeriodic\n";

int loop;

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
		loop = 0;
	}

	void AutonomousInit() {
		serialPort->Write(AutoInitString);
	}

	void AutonomousPeriodic() {
		if (loop % 50 == 0)
			serialPort->Write(AutoPeriodicString);
		loop++;
	}

	void TeleopInit() {
		serialPort->Write(TeleopInitString);
	}

	void TeleopPeriodic() {
		if (loop % 50 == 0)
			serialPort->Write(TeleopPeriodicString);
		loop++;
	}

	void TestInit() {
		serialPort->Write(TestInitString);
	}

	void TestPeriodic() {
		if (loop % 50 == 0)
			serialPort->Write(TestPeriodicString);
		loop++;
	}

private:
	std::shared_ptr<SerialPort> serialPort;
};

START_ROBOT_CLASS(Robot)
