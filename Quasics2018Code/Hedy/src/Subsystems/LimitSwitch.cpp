#include "WPILIB.h"
#include "LimitSwitch.h"
#include "RobotMap.h"
DigitalInput* limitSwitch;
Counter* counter;


LimitSwitch::LimitSwitch() : frc::Subsystem("LimitSwitch"){
     limitSwitch = new frc::DigitalInput(1);
     counter = new Counter(limitSwitch);
}

void LimitSwitch::InitDefaultCommand()
{
}

bool LimitSwitch::CheckSwitch()
{
     return counter == 0;
}

void LimitSwitch::InitializeCounter()
{
     counter = 0;
}
void LimitSwitch::Stop()
{
}

void LimitSwitch::Periodic()
{
}




