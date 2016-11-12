#ifndef MoveAheadTime_H
#define MoveAheadTime_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class MoveAheadTime: public Command

{
private:


		double m_seconds;
		double m_power;
		int counter;
public:

	MoveAheadTime(double seconds, double power );
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
