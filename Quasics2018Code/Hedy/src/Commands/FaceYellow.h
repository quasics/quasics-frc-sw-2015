#ifndef FaceYellow_H
#define FaceYellow_H

#include <WPILib.h>
#include "../Robot.h"

#ifdef VISION_TRACK_CUBES

#include "Commands/Subsystem.h"

class FaceYellow : public frc::Command {
public:
	FaceYellow();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif	// VISION_TRACK_CUBES
#endif  // FaceYellow_H
