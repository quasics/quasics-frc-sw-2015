#ifndef AutoStartLeft_H
#define AutoStartLeft_H

#include <Commands/CommandGroup.h>
#include "iostream"

class AutoStartLeft : public CommandGroup {
public:
	AutoStartLeft();
private:
	std::string gameData;

};

#endif
