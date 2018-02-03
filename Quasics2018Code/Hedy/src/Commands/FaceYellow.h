#ifndef FaceYellow_H
#define FaceYellow_H

#include <WPILib.h>

class FaceYellow : public frc::Command {
public:
	FaceYellow();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif  // FaceYellow_H
