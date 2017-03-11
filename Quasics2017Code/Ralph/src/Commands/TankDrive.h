#ifndef TankDrive_H
#define TankDrive_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class TankDrive : public Command {
public:
	TankDrive();

	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	bool lastButton;
	bool isInverted;
};

#endif  // TankDrive_H
