/*
 * Grabber.h
 *
 *  Created on: Jan 24, 2015
 *      Author: raymond healy
 */

#ifndef SRC_GRABBER_GRABBER_H_
#define SRC_GRABBER_GRABBER_H_

#include "WPILib.h"
#include "ChangeTimes.h"

class Grabber {
public:
	enum Status {
			kWide, kNarrow, kCan
		};
	//Constructor
	Grabber (int screwMotorPort);

	//Commands
	void ChangeSizeStart (Status TargetSize);
	void ChangeSizeProcess ();

	bool IsMoving ();
	bool Wide ();
	bool Narrow ();
	bool Can ();
private:
	Relay screwMotor;
	Timer timer;

	bool moving;
	float targetDuration;
	Status currentStatus;
};

#endif /* SRC_GRABBER_GRABBER_H_ */
