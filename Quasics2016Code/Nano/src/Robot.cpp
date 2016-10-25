#include "WPILib.h"
#include "Robot.h"

	void Robot::RobotInit()
	{

	}

	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
	void Robot::DisabledInit()
	{
	}

	void Robot::DisabledPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void Robot::AutonomousInit()
	{
	}

	void Robot::AutonomousPeriodic()
	{

	}

	void Robot::TeleopInit()
	{
	}

	void Robot::TeleopPeriodic()
	{

	}

	void Robot::TestPeriodic()
	{

	}


