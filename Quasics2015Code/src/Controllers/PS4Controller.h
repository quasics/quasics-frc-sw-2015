/*
 * PS4Controller.h
 *
 *  Created on: Feb 6, 2015
 *      Author: raymond healy
 */

#ifndef SRC_CONTROLLERS_PS4CONTROLLER_H_
#define SRC_CONTROLLERS_PS4CONTROLLER_H_

#include "WPILib.h"
#include "GamepadIOMap.h"
#include <math.h>

class PS4Controller {
public:
	PS4Controller(float input, float DeadbandWidth);

	enum AxisType {
		LeftStickX,
		LeftStickY,
		RightStickX,
		RightStickY,
		LeftTriggerAxis,
		RightTriggerAxis
	};
	enum ButtonType {
		Triangle,
		Square,
		Circle,
		X,
		Share,
		Options,
		Touchpad,
		Logo,
		LeftStick,
		RightStick,
		LeftShoulder,
		RightShoulder,
		LeftTrigger,
		RightTrigger
	};

	void SetDeadband(float NewDeadbandWidth);
	float GetAxis(AxisType axisGet);
	int GetDPad();
	bool GetButton(ButtonType getButton);
private:
	Joystick stick;

	float deadbandWidth;
};

#endif /* SRC_CONTROLLERS_PS4CONTROLLER_H_ */
