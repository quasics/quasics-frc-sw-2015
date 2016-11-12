/*
 * Rotatation.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: dxc101
 */

#include "Rotatation.h"
#include "../Robot.h"

Rotatation::Rotatation(double seconds, double power) : Command(){
	m_seconds = seconds;
	m_power = power;

}
void Rotatation::Initialize() {
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

void Rotatation::Execute() {

	while (m_seconds<= 3){
		Robot::driveTrain->SetLeftPower(-.6);
		Robot::driveTrain->SetRightPower(.6);
	}
	while (m_seconds<= 3){
			Robot::driveTrain->SetLeftPower(0);
			Robot::driveTrain->SetRightPower(0);
	}
	while (m_seconds<= 3){
		Robot::driveTrain->SetLeftPower(-.6);
		Robot::driveTrain->SetRightPower(-.6);
	}
	while (m_seconds<= 3){
			Robot::driveTrain->SetLeftPower(0);
			Robot::driveTrain->SetRightPower(0);
	}
	while (m_seconds<= 3){
		Robot::driveTrain->SetLeftPower(.6);
		Robot::driveTrain->SetRightPower(-.6);
	}
 counter+=1;
}

bool Rotatation::IsFinished() {

	return counter >= m_seconds*50;
	Robot::driveTrain->Stop();
}
void Rotatation::End() {

	Robot::driveTrain->Stop();

}

void Rotatation::Interrupted() {

	Robot::driveTrain->Stop();
}


Rotatation::~Rotatation() {
	// TODO Auto-generated constructor stub

}
