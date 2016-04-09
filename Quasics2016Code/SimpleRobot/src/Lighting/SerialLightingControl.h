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

// Defining functions required by the base class.
protected:
  // CODE_REVIEW(mjh): Consider marking these as "override", rather than just
  // "virtual".  (If the compiler supports it, this will let you make sure that
  // some problems are automatically flagged as compile-time errors, which can
  // help to reveal them early.  It also clarifies where they're "coming from".)
	virtual void SetState(State whichState);
	virtual void SetMode(Mode whichMode);
	virtual void SendHeartbeat();
	virtual void SendBatteryState(bool isLow);
};

#endif /* SRC_LIGHTING_SERIALLIGHTINGCONTROL_H_ */
