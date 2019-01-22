#include "RobotMap.h"
#include <frc/livewindow/LiveWindow.h>

#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>

std::shared_ptr<frc::SpeedController> RobotMap::driveBaseLeftFrontMotor;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseLeftRearMotor;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseLeftMotors;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseRightFrontMotor;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseRightRearMotor;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseRightMotors;


// These are "helper functions", intended to work around some problems in the code
// that Robot Builder currently generates.  (It's still generating code like it
// did for 2017 and earlier, but things are different in 2018.)
template <class Widget>
inline void setNameAndSubsystem(Widget& w, const char* const subsystem, const char* const name) {
	w.SetSubsystem(subsystem);
	w.SetName(name);
}

template <class Controller>
inline std::shared_ptr<Controller> createMotor(int port, const char* const subsystem, const char* const name) {
	std::shared_ptr<Controller> const motor(new Controller(port));
	setNameAndSubsystem(*motor, subsystem, name);
	return motor;
}

inline std::shared_ptr<frc::SpeedControllerGroup> createSpeedControllerGroup(frc::SpeedController& motor1, frc::SpeedController& motor2, const char* const subsystem, const char* const name) {
	std::shared_ptr<frc::SpeedControllerGroup> speedControllerGroup(new frc::SpeedControllerGroup(motor1, motor2));
	setNameAndSubsystem(*speedControllerGroup, subsystem, name);
	return speedControllerGroup;
}

inline std::shared_ptr<frc::Encoder> createEncoder(
		int channelA, int channelB,
		bool reverseDirection, double distancePerPulse,
		frc::PIDSourceType sourceType,
		const char* const subsystem, const char* const name)
{
	std::shared_ptr<frc::Encoder> encoder(new frc::Encoder(channelA, channelB, reverseDirection));
	encoder->SetDistancePerPulse(distancePerPulse);
	encoder->SetPIDSourceType(sourceType);
	setNameAndSubsystem(*encoder, subsystem, name);
	return encoder;
}

void RobotMap::init() {
	//
	// Drive base setup
	driveBaseRightFrontMotor = createMotor<frc::PWMVictorSPX>(0, "Drive Base", "rightFrontMotor");
	driveBaseRightRearMotor = createMotor<frc::PWMVictorSPX>(1, "Drive Base", "rightRearMotor");
    driveBaseLeftFrontMotor = createMotor<frc::PWMVictorSPX>(2, "Drive Base", "leftFrontMotor");
    driveBaseLeftRearMotor = createMotor<frc::PWMVictorSPX>(3, "Drive Base", "leftRearMotor");

    driveBaseLeftMotors = createSpeedControllerGroup(
    		*driveBaseLeftFrontMotor, *driveBaseLeftRearMotor,
			"DriveBase", "LeftDriveMotors");
    driveBaseRightMotors = createSpeedControllerGroup(
    		*driveBaseRightFrontMotor, *driveBaseRightRearMotor,
			"DriveBase", "RightDriveMotors");
}
