#ifndef FaceTape_H
#define FaceTape_H

#include <WPILib.h>
#include "../Robot.h"

#ifndef VISION_TRACK_TAPE

#include "Commands/Subsystem.h"

class FaceTape : public frc::Command {
public:
	FaceTape();
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};


#endif
#endif
