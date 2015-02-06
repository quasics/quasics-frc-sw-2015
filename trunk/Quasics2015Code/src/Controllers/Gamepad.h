/*
 * Gamepad.h
 *
 *  Created on: Jan 23, 2015
 *      Author: ray
 */

#ifndef SRC_CONTROLLERS_GAMEPAD_H_
#define SRC_CONTROLLERS_GAMEPAD_H_

#include "WPILib.h"
#include <math.h>

class Gamepad {
public:
	Gamepad(int input, float DeadbandWidth);

	enum AxisType {
		LeftStickX, LeftStickY, RightStickX, RightStickY
	};
	enum ButtonType {
		A,
		B,
		X,
		Y,
		LeftShoulder,
		RightShoulder,
		LeftTrigger,
		RightTrigger,
		LeftStick,
		RightStick,
		Back,
		Start
	};

	void SetDeadband(float NewDeadbandWidth);
	float GetAxis(AxisType axisGet);
	int GetDPad();
	bool GetButton(ButtonType buttonGet);

private:
	Joystick stick;

	float deadbandWidth;
};

#endif /* SRC_CONTROLLERS_GAMEPAD_H_ */
