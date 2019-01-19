#include "GyroADXRS.h"
#include "../RobotMap.h"

GyroADXRS::GyroADXRS() : frc::Subsystem("GyroADXRS") {
    analogGyro1 = RobotMap::gyroADXRSAnalogGyro1;
}

void GyroADXRS::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

void GyroADXRS::Periodic() {
    // Put code here to be run every loop
}

void GyroADXRS::Reset(){
	analogGyro1->Reset();
}

void GyroADXRS::Calibrate(){
	analogGyro1->Calibrate();
}

double GyroADXRS::GetAngle() {
	return analogGyro1->GetAngle();
}

double GyroADXRS::GetRate() {
	return analogGyro1->GetRate();
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

