#ifndef TankDriveInverted_H
#define TankDriveInverted_H

#include "WPILib.h"

class TankDriveInverted : public Command {
public:
	TankDriveInverted();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TankDriveInverted_H
