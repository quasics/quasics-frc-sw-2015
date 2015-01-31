/*
 * IOMap.h
 *
 *  Created on: Jan 22, 2015
 *      Author: Raymond healy
 */

#ifndef SRC_IOMAP_H_
#define SRC_IOMAP_H_

#include <string>

// Motor port definitions
const int FrontLeftTalonPort = 0;
const int RearLeftTalonPort = 1;
const int FrontRightTalonPort = 8;
const int RearRightTalonPort = 9;
const int LeftElevatorMotorPort = 2;
const int RightElevatorMotorPort = 3;
const int GrabberMotorPort = 4;

const std::string CameraHost("10.26.56.11");

// Encoder definitions
const int LeftEncoderA = 1; //placeholder
const int LeftEncoderB = 2; //placeholder
const int RightEncoderA = 3; //placeholder
const int RightEncoderB = 4; //placeholder

// Gyro input port(s)
const int GyroIn = 0;

// Driver controller port(s)
const int GamePadIn = 1;

#endif /* SRC_IOMAP_H_ */

