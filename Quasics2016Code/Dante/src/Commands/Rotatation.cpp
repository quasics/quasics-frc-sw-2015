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
	counter= 0;
	Requires(Robot::driveTrain.get());

}
void Rotatation::Initialize() {
	counter = 0;
	Robot::driveTrain->SetLeftPower(-m_power);
	Robot::driveTrain->SetRightPower(m_power);
}

void Rotatation::Execute() {


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
