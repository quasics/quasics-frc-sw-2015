#ifndef FuelExhaustGate_H
#define FuelExhaustGate_H

#include "WPILib.h"
#include "../RobotMap.h"

class FuelExhaustGate : public Subsystem {
private:
	std::shared_ptr<Servo> outputActuator;

		const float maxValue = 1.0;
		const float openValue = maxValue;
		const float minValue = 0;
		const float closeValue = minValue;

		bool doorOpen;
public:
	FuelExhaustGate();
	virtual ~FuelExhaustGate();

		void Set(bool isOpen);
		bool Get();
		double GetAngle();
};

#endif  // FuelExhaustGate_H
