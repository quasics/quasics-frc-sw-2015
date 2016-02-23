#include "Robot.h"
#include <iostream>

ProtoMan::ProtoMan() {
	CameraServer::GetInstance()->SetQuality(50);
	//the camera name (ex "cam0") can be found through the roborio web interface
	CameraServer::GetInstance()->StartAutomaticCapture("cam0");
}

void ProtoMan::RobotInit() {

}

void ProtoMan::AutonomousInit() {

}

void ProtoMan::AutonomousPeriodic() {

}
void ProtoMan::TeleopInit() {

}

void ProtoMan::TeleopPeriodic() {
}
void ProtoMan::TestInit() {

}



void ProtoMan::TestPeriodic() {

}

START_ROBOT_CLASS(ProtoMan);
