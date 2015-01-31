/*
 * Grabber.h
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */

#ifndef SRC_GRABBER_GRABBER_H_
#define SRC_GRABBER_GRABBER_H_

#include "WPILib.h"

class Grabber {
public:
	//Constructor
	Grabber (int screwMotorPort);

	//Commands
private:
	Relay screwMotor;

};

#endif /* SRC_GRABBER_GRABBER_H_ */
