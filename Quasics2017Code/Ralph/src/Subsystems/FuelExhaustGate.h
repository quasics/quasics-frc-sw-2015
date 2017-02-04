#ifndef FuelExhaustGate_H
#define FuelExhaustGate_H

#include "WPILib.h"
#include "../RobotMap.h"

class FuelExhaustGate : public Subsystem {
private:
	enum DoorState {
		eOpen,
		eClosed,
		eAjar
	};
	std::shared_ptr<Servo> outputActuator;

	const float openValue = 1.0;
	const float closeValue = 0;

	bool doorOpen;
public:
	FuelExhaustGate();
	virtual ~FuelExhaustGate();

	void Set(bool isOpen);
	bool Get() const;
	DoorState GetDoorStatus() const;
};

#endif  // FuelExhaustGate_H
