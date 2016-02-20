/*
 * LightingControl.h
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#ifndef SRC_LIGHTING_SERIALLIGHTINGCONTROL_H_
#define SRC_LIGHTING_SERIALLIGHTINGCONTROL_H_

#include "LightingControl.h"

class SerialLightingControl: public LightingControl {
public:
	SerialLightingControl();
protected:
	static std::unique_ptr<SerialPort> serialPort;

	virtual void SetState(State whichState);
	virtual void SetMode(Mode whichMode);
	virtual void SendHeartbeat();
	virtual void SendBatteryState(bool isLow);
};

#endif /* SRC_LIGHTING_SERIALLIGHTINGCONTROL_H_ */
