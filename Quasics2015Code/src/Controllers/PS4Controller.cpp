/*
 * PS4Controller.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: raymond healy
 */
#include "PS4Controller.h"

PS4Controller::PS4Controller(float input, float DeadbandWidth) :
		stick(input, 6, 14), deadbandWidth(DeadbandWidth) {

}

void PS4Controller::SetDeadband(float newDeadbandWidth) {
	deadbandWidth = newDeadbandWidth;
}

float PS4Controller::GetAxis(AxisType getAxis) {
	float toReturn;
	switch (getAxis) {
	case LeftStickX:
		toReturn = stick.GetRawAxis(PS4LeftStickXAxis);
		break;
	case LeftStickY:
		toReturn = stick.GetRawAxis(PS4LeftStickYAxis);
		break;
	case RightStickX:
		toReturn = stick.GetRawAxis(PS4RightStickXAxis);
		break;
	case RightStickY:
		toReturn = stick.GetRawAxis(PS4RightStickYAxis);
		break;
	case LeftTriggerAxis:
		toReturn = stick.GetRawAxis(PS4LeftTriggerAxis);
		break;
	case RightTriggerAxis:
		toReturn = stick.GetRawAxis(PS4RightTriggerAxis);
		break;
	}
	if(fabs(toReturn) <= deadbandWidth){
		toReturn = 0;
	}
	return toReturn;
}

int PS4Controller::GetDPad(){
	return(stick.GetPOVCount());
}

bool PS4Controller::GetButton(ButtonType getButton){
	switch (getButton){
	case Triangle:
		return stick.GetRawButton(PS4TriangleButton);
	case Square:
		return stick.GetRawButton(PS4SquareButton);
	case Circle:
		return stick.GetRawButton(PS4CircleButton);
	case X:
		return stick.GetRawButton(PS4XButton);
	case Share:
		return stick.GetRawButton(PS4ShareButton);
	case Options:
		return stick.GetRawButton(PS4OptionsButton);
	case Touchpad:
		return stick.GetRawButton(PS4SRCPress);
	case Logo:
		return stick.GetRawButton(PS4LogoButton);
	case LeftStick:
		return stick.GetRawButton(PS4LeftStickPress);
	case RightStick:
		return stick.GetRawButton(PS4RightStickPress);
	case LeftShoulder:
		return stick.GetRawButton(PS4LeftShoulder);
	case RightShoulder:
		return stick.GetRawButton(PS4RightShoulder);
	case LeftTrigger:
		return stick.GetRawButton(PS4LeftTrigger);
	case RightTrigger:
		return stick.GetRawButton(PS4RightTrigger);
	default:
		return false;
	}
}
