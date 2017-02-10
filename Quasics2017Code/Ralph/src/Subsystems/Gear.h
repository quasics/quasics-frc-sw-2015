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
	const float openValue = .25;
	const float closeValue = .65;
	bool doorOpen;

public:
	Gear();
	virtual ~Gear();

	void Set(bool isOpen);
	bool Get();
	double GetPosition ();

};

#endif
