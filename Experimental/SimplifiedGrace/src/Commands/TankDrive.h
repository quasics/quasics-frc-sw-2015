#ifndef TANK_DRIVE_H
#define TANK_DRIVE_H

#include <cstdint>
#include <Commands/Command.h>

class TankDrive : public frc::Command {
public:
	TankDrive();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	int counter = 0;
	bool pressedLastTime = false;
};

#endif  // TANK_DRIVE_H
