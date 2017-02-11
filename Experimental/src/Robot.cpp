#include <iostream>
#include <memory>
#include <string>

#include "WPILib.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
	}
	std::string serialText = "Smoke Weed Everyday";

	void AutonomousInit() {
	}
	int loop = 0;
	void AutonomousPeriodic() {
		if (loop%50 == 0)
		serialPort->Write("Smoke Weed Everyday\n");
		loop++;
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

	}

	void TestPeriodic() {

	}

private:
	std::shared_ptr<SerialPort> serialPort;
};

START_ROBOT_CLASS(Robot)
