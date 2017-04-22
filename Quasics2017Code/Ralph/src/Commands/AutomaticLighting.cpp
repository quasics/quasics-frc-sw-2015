#include "AutomaticLighting.h"
#include "SetLightColor.h"
#include "SetLightDynamics.h"
#include "../Subsystems/Arduino.h"

AutomaticLighting::AutomaticLighting() {
	if (DriverStation::GetInstance().IsAutonomous()) {
		AddSequential(new SetLightColor(Arduino::kTeam));
		AddSequential(new SetLightDynamics(Arduino::kBlinking));
	} else if (DriverStation::GetInstance().IsOperatorControl()
			&& DriverStation::GetInstance().GetMatchTime() > 30) {
		AddSequential(new SetLightColor(Arduino::kTeam));
		AddSequential(new SetLightDynamics(Arduino::kDashed));
	} else if (DriverStation::GetInstance().IsOperatorControl()) {
		AddSequential(new SetLightColor(Arduino::kTeam));
		AddSequential(new SetLightDynamics(Arduino::kRolling));
	} else if (DriverStation::GetInstance().IsDisabled()) {
		AddSequential(new SetLightColor(Arduino::kTeam));
		AddSequential(new SetLightDynamics(Arduino::kBreathing));
	} else if (DriverStation::GetInstance().IsTest()) {
		AddSequential(new SetLightColor(Arduino::kYellow));
		AddSequential(new SetLightDynamics(Arduino::kOn));
	} else {
		AddSequential(new SetLightColor(Arduino::kRainbow));
		AddSequential(new SetLightDynamics(Arduino::kOn));
	}
}
