/*
 * LightingControl.h
 *
 *  Created on: Feb 6, 2016
 *      Author: rmh102
 */

#ifndef SRC_LIGHTING_LIGHTINGCONTROL_H_
#define SRC_LIGHTING_LIGHTINGCONTROL_H_

#include "WPILib.h"

class LightingControl {
public:
	LightingControl();
	virtual ~LightingControl() {}

	/** Main logic control goes here, allowing this function to handle
	 * common decision-making.  Actual communications with the Arduino
	 * is handled by the protected functions (below), and will be
	 * implemented in derived classes, allowing the decision of the
	 * communications channel being used (serial port, I2C, etc.) to
	 * be deferred.
	 */
	void LightingUpkeep ();

private:
	// State used in the "upkeep" function.
	int batteryTimer;
	int heartbeatTimer;
	bool isBatteryLow;

// Stuff dealing with actual communications with the Arduino,
// which will be *really* implemented in derived classes.
protected:
	enum State {
		kRedTeam = 0, kBlueTeam = 1, kDemo = 2, kError = 3
	};
	enum Mode {
		kSolid = 0, kSlowBlinking = 1, kMediumBlink = 2, kQuickBlink = 3, kBreathing = 4
	};

	friend std::ostream& operator<<(std::ostream& os, State s);
	friend std::ostream& operator<<(std::ostream& os, Mode m);

	virtual void SetState (State whichState) = 0;
	virtual void SetMode (Mode whichMode) = 0;
	virtual void SendHeartbeat () = 0;
};

class SimulatedLightingControl : public LightingControl {
public:
	SimulatedLightingControl() {}
protected:
	virtual void SetState (State whichState);
	virtual void SetMode (Mode whichMode);
	virtual void SendHeartbeat ();
};

#endif /* SRC_LIGHTING_LIGHTINGCONTROL_H_ */
