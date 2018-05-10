
#include "AutoModeRight.h"
#include "Robot.h"
#include <DriverStation.h>
#include "Commands/AutoRightSwitch.h"
#include "Commands/AutoExchange.h"
#include "Primitives/MoveForDistance.h"



AutoModeRight::AutoModeRight(): ConditionalCommand(new AutoRightSwitch, new MoveForDistance(100, -.3)) {
}

bool AutoModeRight::Condition(){

	const std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if (gameData.empty()){
		return false;
	}

	bool shouldWeScoreOnSwitch = false;

	const bool ourSideIsRightOnSwitch = (gameData[0] == 'R');

	if (ourSideIsRightOnSwitch) {
		shouldWeScoreOnSwitch = true;
		//Switch
	}
	else {
		shouldWeScoreOnSwitch = false;
		//Straight
	}

	return shouldWeScoreOnSwitch;
}
