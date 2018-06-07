#include "MoveForDistance.h"
#include "../../Robot.h"
#include <iostream>
#include <iomanip>

#define NOISY

#ifdef NOISY
#define LOG(x)	do { std::cout << x << std::endl; } while(false)
#else
#define LOG(x)
#endif

MoveForDistance::MoveForDistance(uint32_t targetInches, double powerLevel)
: frc::Command(), target(targetInches), power(powerLevel), leftPower(0), rightPower(0) {
	Requires (Robot::driveBase.get());
}

// Called just before this Command runs the first time
void MoveForDistance::Initialize() {
	Robot::driveBase->RightEncoderReset();
	Robot::driveBase->LeftEncoderReset();
	leftPower = power /* * 0.95 */;
	rightPower = power;
	Robot::driveBase->SetPowerToMotors(-(leftPower), rightPower);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDistance::Execute() {
	static constexpr double deltaDown = 0.95;
	static constexpr double deltaUp = 1.04;
	static constexpr double maxOffset = 1.2;
	static constexpr double minOffset = 0.8;

	const double leftDistance = fabs(Robot::driveBase->LeftEncoderDistance());
	const double rightDistance = fabs(Robot::driveBase->RightEncoderDistance());

	LOG("Left dist: " << std::setw(5) << leftDistance
		  << " Right dist: " << std::setw(5) << rightDistance
		  << " / Left power: " << std::setw(5) << leftPower
		  << " Right power: " << std::setw(5) << rightPower);
	if(leftDistance  > rightDistance) {
		double v = leftPower * deltaDown;
		if (fabs(v) >= fabs(power * minOffset)) {
			leftPower = v;
		} else {
			if (fabs(rightPower * deltaUp) <= fabs(power * maxOffset)) {
				rightPower = rightPower * deltaUp;
			} else {
				rightPower = power * maxOffset;
			}
		}		// rightPower *= 1.1;
		Robot::driveBase->SetPowerToMotors(-leftPower, rightPower);
	}
	else if(leftDistance < rightDistance){
		double v = rightPower * deltaDown;
		if (fabs(v) >= fabs(power * .8)) {
			rightPower = v;
		} else {
			if (fabs(leftPower * deltaUp) <= fabs(power * 1.2)) {
				leftPower = leftPower * deltaUp;
			} else {
				leftPower = power * 1.2;
			}
		}
		Robot::driveBase->SetPowerToMotors(-(leftPower), (rightPower));
	}
	else{
		Robot::driveBase->SetPowerToMotors(-(leftPower .2ZE21``````W-), rightPower);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool MoveForDistance::IsFinished() {
	return (fabs(Robot::driveBase->LeftEncoderDistance()) > target
			|| fabs(Robot::driveBase->RightEncoderDistance()) > target);
}

// Called once after isFinished returns true
void MoveForDistance::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDistance::Interrupted() {
	End();
}
