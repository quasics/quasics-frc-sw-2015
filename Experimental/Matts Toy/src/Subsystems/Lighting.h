#ifndef LIGHTING_H
#define LIGHTING_H

#include <WPILib.h>

/**
 *
 *
 * @author ExampleAuthor
 */
class Lighting: public Subsystem {
public:
	Lighting();
	void InitDefaultCommand();

	enum Alliance {
		eRed = 0, eBlue = 1, eDemo = 2
	};
	void setAlliance(Alliance alliance);

	enum Mode {
		eIdle = 0, eTeleOp = 1, eAuto = 2, eError = 3, eTest = 4
	};
	void setMode(Mode mode);

	void updateState(bool force = false);

protected:
	void sendLightingCommand(bool force);

private:
	std::shared_ptr<SerialPort> serialPort_;
	Alliance alliance_;
	Mode mode_;
	const char* lastCommand_ = nullptr;
};

#endif
