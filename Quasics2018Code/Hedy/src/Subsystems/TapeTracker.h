#ifndef TapeTracker_H
#define TapeTracker_H

#include <Commands/Subsystem.h>

class TapeTracker : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	TapeTracker();
	void InitDefaultCommand();
};

#endif  // TapeTracker_H
