/*
 * Gear.h
 *
 *  Created on: Jan 27, 2017
 *      Author: axf105
 */

#ifndef SRC_SUBSYSTEMS_GEAR_H_
#define SRC_SUBSYSTEMS_GEAR_H_

#include <WPILib.h>

class Gear : public Subsystem {
private:
	std::shared_ptr<Servo> gearServo;

	const float maxValue = 1.0;
	const float openValue = maxValue;
	const float minValue = 0;
	const float closeValue = minValue;

	bool doorOpen;

public:
	Gear();
	virtual ~Gear();

	void Set(bool isOpen);
	bool Get();

};

#endif
