#include "TapeTracker.h"
#include <iostream>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define WIDTH_SCALING
#define HEIGHT_SCALING
TapeTracker::TapeTracker() : frc::Subsystem("TapeTracker") {

}

void TapeTracker::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
