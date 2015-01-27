/*
 * Gamepad.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: ray
 */
#include "Controller/ControllerIOMap.h"
#include "Controller/Gamepad.h"

Gamepad::Gamepad(int input, float initialDeadbandWidth) :
		stick(input, 4, 12), deadbandWidth(initialDeadbandWidth) {
}

void Gamepad::SetDeadband(float NewDeadbandWidth) {
	deadbandWidth = NewDeadbandWidth;
}

float Gamepad::GetAxis(AxisType axisGet) {
	float toReturn;
	switch (axisGet) {
	case LeftStickX:
		toReturn = stick.GetRawAxis(LeftStickXAxis);
		break;
	case LeftStickY:
		toReturn = stick.GetRawAxis(LeftStickYAxis);
		break;
	case RightStickX:
		toReturn = stick.GetRawAxis(RightStickXAxis);
		break;
	case RightStickY:
		toReturn = stick.GetRawAxis(RightStickYAxis);
		break;
	}
	if (fabs(toReturn) <= deadbandWidth) {
		toReturn = 0;
	}
	return (toReturn);
}
int Gamepad::GetDPad (){
	return (stick.GetPOV());
}

bool Gamepad::GetButton(ButtonType buttonGet) {
	switch (buttonGet) {
	case A:
		return (stick.GetRawButton(AButton));
	case B:
		return (stick.GetRawButton(BButton));
	case X:
		return (stick.GetRawButton(XButton));
	case Y:
		return (stick.GetRawButton(YButton));
	case LeftShoulder:
		return (stick.GetRawButton(LeftShoulderButton));
	case RightShoulder:
		return (stick.GetRawButton(RightShoulderButton));
	case LeftStick:
		return (stick.GetRawButton(LeftStickPress));
	case RightStick:
		return (stick.GetRawButton(RightStickPress));
	case Back:
		return (stick.GetRawButton(BackButton));
	case Start:
		return (stick.GetRawButton(StartButton));
	default:
		return false;
	}
}
