/*
 * LightingControl.h
 *
 *  Created on: Nov 7, 2017
 *      Author: healym
 */

#ifndef SRC_SUBSYSTEMS_LIGHTINGCONTROL_H_
#define SRC_SUBSYSTEMS_LIGHTINGCONTROL_H_

#include <Commands/Subsystem.h>
#include <WPILib.h>

class LightingControl: public frc::Subsystem {
public:
	LightingControl();

	void InitDefaultCommand();

	//
	// Functions for manual configuration of lighting in Commands
	enum Alliance {
		eRed = 0, eBlue = 1, eDemo = 2
	};
	enum Mode {
		eIdle = 0, eTeleOp = 1, eAuto = 2, eError = 3, eTest = 4
	};

	void setAlliance(Alliance alliance);
	void setMode(Mode mode);

	//
	// Functions for automatic configuration of lighting in Commands
	// (e.g., in the AutomaticLightingCommand class).
	void updateState(bool force = false);

protected:
	/**
	 * Sends a command to the Arduino, indicating the lighting mode to
	 * be used.
	 *
	 * @param force if false, the command will only be sent if we think that
	 *              the lighting mode has changed from the last time that it
	 *              was sent out.
	 */
	void sendLightingCommand(bool force);

private:
	std::shared_ptr<SerialPort> serialPort_;
	Alliance alliance_;
	Mode mode_;
	const char* lastCommand_ = nullptr;
};

#endif /* SRC_SUBSYSTEMS_LIGHTINGCONTROL_H_ */
