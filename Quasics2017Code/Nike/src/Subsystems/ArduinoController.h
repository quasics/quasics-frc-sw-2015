#ifndef ArduinoController_H
#define ArduinoController_H

#include <Commands/Subsystem.h>

class ArduinoController : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	ArduinoController();
	void InitDefaultCommand();
};

#endif  // ArduinoController_H
