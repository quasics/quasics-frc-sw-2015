// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#ifndef INTAKE_H
#define INTAKE_H

#include <Commands/Subsystem.h>
#include <WPILib.h>

#define USE_INTAKE_BAR_

/**
 *
 *
 * @author ExampleAuthor
 */
class Intake: public Subsystem {
#ifndef USE_INTAKE_BAR_
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<SpeedController> leftIntakeWheel;
	std::shared_ptr<SpeedController> rightIntakeWheel;
	std::shared_ptr<Relay> pusher;
public:
	Intake();
	void InitDefaultCommand();

	enum Direction {
		kIntake, kOutput, kOff
	};

	void SetPower (Direction whichDirection);
	void StopIntake ();
	void CamGearControl (bool isOn);
#else
public:
	Intake();
	void InitDefaultCommand();

	enum Direction {
		kIntake, kOutput, kOff
	};

	void SetPower (Direction whichDirection);
	void StopIntake ();

private:
	std::shared_ptr<SpeedController> leftIntakeWheel;
#endif
};

#endif
