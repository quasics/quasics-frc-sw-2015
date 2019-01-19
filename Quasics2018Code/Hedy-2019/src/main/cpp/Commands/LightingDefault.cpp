/*
 * LightingDefault.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: sth101
 */

#include "LightingDefault.h"
#include "Robot.h"
LightingDefault::LightingDefault() {
	// TODO Auto-generated constructor stub
	Requires(Robot::lighting.get());
}
void LightingDefault::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void LightingDefault::Execute() {
	 char status = Robot::lighting->GetMode();
	 char color = Robot::lighting->GetColor();
	 if(status == 'a'){
		 teleOpCounter = -1;
		 autoCounter = autoCounter + 1;
	 }
	 else if(status == 't'){
		 autoCounter = -1;
		 teleOpCounter = teleOpCounter + 1;
	 }
	 else{
		 autoCounter = 0;
		 teleOpCounter = 0;
	 }

	 if(teleOpCounter % 150 == 0){
		 Robot::lighting->WriteTeleOp();
	 }
	 else if(autoCounter % 100 == 0){
		 Robot::lighting->WriteAuto();
	 }
	 else{
	 }

	 if(autoCounter == 0){
		 if(color == 'r'){
			 Robot::lighting->WriteRed();
		 }
		 else if(color == 'b'){
			 Robot::lighting->WriteBlue();
		 }
		 else{
			 Robot::lighting->WriteGreen();
		 }
	 }
}

// Make this return true when this Command no longer needs to run execute()
bool LightingDefault::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void LightingDefault::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void LightingDefault::Interrupted() {

}


