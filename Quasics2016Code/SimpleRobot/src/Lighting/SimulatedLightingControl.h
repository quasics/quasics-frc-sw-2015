/*
 * LightingControl.h
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#ifndef SRC_LIGHTING_SIMULATEDLIGHTINGCONTROL_H_
#define SRC_LIGHTING_SIMULATEDLIGHTINGCONTROL_H_

#include "LightingControl.h"

class SimulatedLightingControl: public LightingControl {
public:
	SimulatedLightingControl() {}

protected:
	virtual void SetState(State whichState);
	virtual void SetMode(Mode whichMode);
	virtual void SendHeartbeat();
	virtual void SendBatteryState(bool isLow);
};

#endif /* SRC_LIGHTING_SIMULATEDLIGHTINGCONTROL_H_ */
