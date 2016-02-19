#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_
#include "WPILib.h"
#include "SerialPort.h"

class ProtoMan: public IterativeRobot {
public:
	ProtoMan();
	std::unique_ptr<SerialPort> serialPort;

private:


	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestInit();
	void TestPeriodic ();

};

#endif /* SRC_ROBOT_H_ */
