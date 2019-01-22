

#ifndef AUTO_MODE_RIGHT_H
#define AUTO_MODE_RIGHT_H



#include "frc/commands/ConditionalCommand.h"



class AutoModeRight : public frc::ConditionalCommand {
public:

	AutoModeRight();


protected:
virtual bool Condition();
};

#endif
