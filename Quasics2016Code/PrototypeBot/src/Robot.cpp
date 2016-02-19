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

std::string m_text = ";blue;disabled;";
void ProtoMan::TeleopPeriodic() {
	static unsigned int curPos = 0;
	if (curPos < m_text.length()) {
			const std::string toWrite(m_text.c_str() + curPos);
			const uint32_t bytesWritten = serialPort->Write(toWrite, toWrite.length());
			curPos += bytesWritten;
			std::cout << "Wrote " << curPos << " bytes of '" << toWrite << "'" << std::endl;
		}
}
void ProtoMan::TestInit() {

}



void ProtoMan::TestPeriodic() {

}

START_ROBOT_CLASS(ProtoMan);
