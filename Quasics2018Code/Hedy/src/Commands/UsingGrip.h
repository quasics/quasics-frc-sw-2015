#ifndef UsingGrip_H
#define UsingGrip_H

#include <WPILib.h>

class UsingGrip : public frc::Command {
public:
	UsingGrip();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // UsingGrip_H
