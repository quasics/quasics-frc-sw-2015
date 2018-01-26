#ifndef AutoStartRight_H
#define AutoStartRight_H

#include <Commands/CommandGroup.h>
#include "iostream"

class AutoStartRight : public CommandGroup {
public:
	AutoStartRight();
private:
	std::string gameData;

};

#endif
