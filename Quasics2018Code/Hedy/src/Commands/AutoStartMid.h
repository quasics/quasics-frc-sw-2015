#ifndef AutoStartMid_H
#define AutoStartMid_H

#include <Commands/CommandGroup.h>
#include "iostream"

class AutoStartMid : public CommandGroup {
public:
	AutoStartMid();
private:
	std::string gameData;

};

#endif
