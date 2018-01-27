#include "AutoStartLeft.h"
#include "AutoSwitchLeft.h"
#include "AutoSwitchRight.h"
#include "AutoFMS.h"
#include "MoveForDistance.h"
AutoStartLeft::AutoStartLeft() {
	AddSequential(new AutoFMS());
	if(gameData[0] == 'L'){
		//left
		AddSequential(new AutoSwitchLeft());
	}
	else {
		//right
		AddSequential(new MoveForDistance(192, .4));
	}
}
