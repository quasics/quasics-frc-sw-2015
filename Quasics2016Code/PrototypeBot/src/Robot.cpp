#include "Robot.h"
#include <iostream>

ProtoMan::ProtoMan() {
	serialPort.reset(new SerialPort(115200, SerialPort::kMXP));
}

void ProtoMan::RobotInit() {

}

void ProtoMan::AutonomousInit() {

}

void ProtoMan::AutonomousPeriodic() {

}
void ProtoMan::TeleopInit() {

}

const std::string sampleSerialText = ";blue;disabled;";

void ProtoMan::TeleopPeriodic() {
}
void ProtoMan::TestInit() {
	const uint32_t bytesWritten = serialPort->Write(sampleSerialText, sampleSerialText.length());
	std::cout << "Wrote " << bytesWritten << " bytes of '" << sampleSerialText << "'" << std::endl;
}



void ProtoMan::TestPeriodic() {

}

START_ROBOT_CLASS(ProtoMan);
