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
};

#endif  // TANK_DRIVE_H
