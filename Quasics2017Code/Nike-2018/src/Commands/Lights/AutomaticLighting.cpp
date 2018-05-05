#include "AutomaticLighting.h"

AutomaticLighting::AutomaticLighting() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(Robot::arduinoController.get());
	batteryTimer = 0;
	color = ArduinoController::kGreen;
	dynamic = ArduinoController::kOn;
}

// Called just before this Command runs the first time
void AutomaticLighting::Initialize() {
	batteryTimer = 0;
	color = ArduinoController::kGreen;
	dynamic = ArduinoController::kOn;
}

const double lowBatteryThreshold = 11;
const uint32_t lowBatteryCounts = 50;

// Called repeatedly when this Command is scheduled to run
void AutomaticLighting::Execute() {
	//------------------------------Battery Check------------------------------
	if (DriverStation::GetInstance().GetBatteryVoltage()
			< lowBatteryThreshold) {
		batteryTimer++;
	} else {
		batteryTimer = 0;
	}
	bool isLowBattery = batteryTimer > lowBatteryCounts;
	//-------------------------------------------------------------------------

	//-------------------------------Color Tree--------------------------------
	if (isLowBattery) {
		color = ArduinoController::kYellow;
	} else if (!DriverStation::GetInstance().IsFMSAttached()) {
		color = ArduinoController::kGreen;
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::kRed) {
		color = ArduinoController::kRed;
	} else if (DriverStation::GetInstance().GetAlliance()
			== DriverStation::kBlue) {
		color = ArduinoController::kBlue;
	} else {
		color = ArduinoController::kRainbow;
	}
	//-------------------------------------------------------------------------

	//---------------------------Dynamic Tree----------------------------------
	if (isLowBattery) {
		dynamic = ArduinoController::kSnake;
	} else if (DriverStation::GetInstance().IsEnabled()
			&& DriverStation::GetInstance().IsAutonomous()) {
		dynamic = ArduinoController::kBlinking;
	} else if (DriverStation::GetInstance().IsEnabled()
			&& DriverStation::GetInstance().IsOperatorControl()) {
		dynamic = ArduinoController::kDashed;
	} else {
		dynamic = ArduinoController::kOn;
	}
	//-------------------------------------------------------------------------

	Robot::arduinoController->SetLightColor(color);
	Robot::arduinoController->SetLightDynamic(dynamic);
}

// Make this return true when this Command no longer needs to run execute()
bool AutomaticLighting::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutomaticLighting::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutomaticLighting::Interrupted() {

}
