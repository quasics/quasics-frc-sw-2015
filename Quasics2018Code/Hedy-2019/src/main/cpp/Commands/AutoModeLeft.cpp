
#include "AutoModeLeft.h"
#include "Robot.h"
#include <frc/DriverStation.h>
#include "Commands/AutoLeftSwitch.h"
#include "Commands/AutoExchange.h"



AutoModeLeft::AutoModeLeft(): ConditionalCommand(new AutoLeftSwitch, new AutoExchange) {
}

bool AutoModeLeft::Condition(){

	const std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if (gameData.empty()){
		return false;
	}

	bool shouldWeScoreOnSwitch = false;

	const bool ourSideIsLeftOnSwitch = (gameData[0] == 'L');

	if (ourSideIsLeftOnSwitch) {
		shouldWeScoreOnSwitch = true;
		//Switch
	}
	else {
		shouldWeScoreOnSwitch = false;
		//Exchange
	}

	return shouldWeScoreOnSwitch;
}
