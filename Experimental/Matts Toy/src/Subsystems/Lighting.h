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
		eIdle = 0, eTeleOp = 1, eAuto = 2, eError = 3
	};
	void setMode(Mode mode);

protected:
	void sendLightingCommand();

private:
	std::shared_ptr<SerialPort> serialPort_;
	Alliance alliance_;
	Mode mode_;
};

#endif
