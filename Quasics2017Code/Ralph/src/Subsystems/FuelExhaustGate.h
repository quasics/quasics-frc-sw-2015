#ifndef FuelExhaustGate_H
#define FuelExhaustGate_H

#include "WPILib.h"
#include "../RobotMap.h"

class FuelExhaustGate : public Subsystem {
private:
	std::shared_ptr<Servo> outputActuator;

	bool doorOpen;
public:
	FuelExhaustGate();
	virtual ~FuelExhaustGate();

	void Set(bool isOpen);
	bool Get() const;
	double GetPosition() const { return outputActuator->GetAngle(); }

	enum DoorState {
		eOpen,
		eClosed,
		eAjar
	};
	DoorState GetDoorStatus() const;
};

#endif  // FuelExhaustGate_H
