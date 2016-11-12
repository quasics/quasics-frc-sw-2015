/*
 * Rotate.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: axf105
 */

#include <Commands/Rotate.h>
#include "../Robot.h"

Rotate::Rotate(double seconds, double power): Command() {
	m_seconds = seconds;
	m_power = power;
}

void Rotate::Initialize() {
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

void Rotate::Execute() {

	while (m_seconds <= 3){
		Robot::driveTrain->SetLeftPower(.6);
		Robot::driveTrain->SetRightPower(-.6);
	}
	counter+=1;
}

bool Rotate::IsFinished() {
	return counter >= m_seconds*50;
}

void Rotate::End() {

	Robot::driveTrain->Stop();
}

void Rotate:: Interrupt() {

	Robot::driveTrain->Stop();

}

Rotate::~Rotate() {
	// TODO Auto-generated destructor stub
}

