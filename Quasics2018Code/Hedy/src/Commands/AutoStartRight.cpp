#include "AutoStartRight.h"
#include "AutoSwitchLeft.h"
#include "AutoSwitchRight.h"
#include "AutoFMS.h"
#include "MoveForDistance.h"
AutoStartRight::AutoStartRight() {
	AddSequential(new AutoFMS());
	if(gameData[0] == 'R'){
		//left
		AddSequential(new AutoSwitchRight());
	}
	else {
		//right
		AddSequential(new MoveForDistance(192, .4));
	}
}
