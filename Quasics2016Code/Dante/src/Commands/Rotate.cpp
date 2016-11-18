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
	counter = 0;
	Requires (Robot::driveTrain.get());
}

void Rotate::Initialize() {
	counter = 0;
	Robot::driveTrain->SetLeftPower(m_power);
	Robot::driveTrain->SetRightPower(-m_power);
}

void Rotate::Execute() {
	counter+=1;
}

bool Rotate::IsFinished() {
	return counter >= m_seconds*50;
	Robot::driveTrain->Stop();
}

void Rotate::End() {

	Robot::driveTrain->Stop();
}

void Rotate:: Interrupted() {

	Robot::driveTrain->Stop();

}

Rotate::~Rotate() {
	// TODO Auto-generated destructor stub
}

