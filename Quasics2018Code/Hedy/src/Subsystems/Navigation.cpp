#include "Navigation.h"
#include "../RobotMap.h"
#include <iostream>

Navigation::Navigation() : frc::Subsystem("Navigation") {
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

bool Navigation::isReady() {
	return (ahrs.get() != nullptr)
			&& ahrs->IsConnected()
			&& !ahrs->IsCalibrating();
}

void Navigation::resetBearing() {
	if (ahrs.get() != nullptr) {
		ahrs->ZeroYaw();
	}
}

float Navigation::getBearing() {
	if (isReady()) {
		return ahrs->GetYaw();
	} else {
		std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
		return 0;
	}
}

float Navigation::getCompassHeading() {
	if (isReady()) {
		return ahrs->GetCompassHeading();
	}
	else {
		std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
		return 0;
	}
}

float Navigation::getAngle() {
	if (isReady()) {
		return ahrs->GetAngle();
	}
	else {
		std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
		return 0;
	}
}
