

#ifndef AUTO_MODE_LEFT_H
#define AUTO_MODE_LEFT_H



#include "Commands/ConditionalCommand.h"



class AutoModeLeft : public frc::ConditionalCommand {
public:

	AutoModeLeft();


protected:
virtual bool Condition();
};

#endif
