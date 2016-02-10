#include "AutonomousCommand.h"

bool AutonomousCommand::Auto1() {
	static bool isFinished = false;

	switch (autoStage) {
	case 0:
		Robot::driveSystem->ResetEncoders();
		Robot::driveSystem->ResetYaw();
		autoStage = 1;
		break;
	case 1:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 1.34) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 4;
			//reset Sensors
		}
		break;
	case 2:
		Robot::driveSystem->MoveLeft(50);
		Robot::driveSystem->MoveRight(-50);

		if (Robot::driveSystem->GetContinuousYaw() >= -30) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 5;
		}
		break;
	case 3:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 2.83) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			isFinished = true;
			//reset Sensors
		}
		break;

	default:
		isFinished = true;
	}
	return isFinished;
}

bool AutonomousCommand::Auto2() {
	static bool isFinished = false;

	switch (autoStage) {
	case 0:
		Robot::driveSystem->ResetEncoders();
		Robot::driveSystem->ResetYaw();
		autoStage = 1;
		break;
	case 1:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 1.95) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 4;
			//reset Sensors
		}
		break;
	case 2:
		Robot::driveSystem->MoveLeft(50);
		Robot::driveSystem->MoveRight(-50);

		if (Robot::driveSystem->GetContinuousYaw() >= -30) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 5;
		}
		break;
	case 3:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 2.53) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			isFinished = true;
			//reset Sensors
		}
		break;
	default:
		isFinished = true;
	}

	return isFinished;
}

bool AutonomousCommand::Auto3() {
	static bool isFinished = false;

	switch (autoStage) {
	case 0:
		Robot::driveSystem->ResetEncoders();
		Robot::driveSystem->ResetYaw();
		autoStage = 1;
		break;
	case 1:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= .731) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 2;
			//reset Sensors
		}
		break;
	case 2:
		Robot::driveSystem->MoveLeft(-50);
		Robot::driveSystem->MoveRight(50);

		if (Robot::driveSystem->GetContinuousYaw() <= 60) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 3;
		}
		break;
	case 3:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 1.58) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 4;
			//reset Sensors
		}
		break;
	case 4:
		Robot::driveSystem->MoveLeft(50);
		Robot::driveSystem->MoveRight(-50);

		if (Robot::driveSystem->GetContinuousYaw() >= -60) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 5;
		}
		break;
	case 5:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= .4572) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			isFinished = true;
			//reset Sensors
		}
		break;
	default:
		isFinished = true;
	}

	return isFinished;
}

bool AutonomousCommand::Auto4() {
	static bool isFinished = false;

	switch (autoStage) {
	case 0:
		Robot::driveSystem->ResetEncoders();
		Robot::driveSystem->ResetYaw();
		autoStage = 1;
		break;
	case 1:
		Robot::driveSystem->MoveLeft(50);
		Robot::driveSystem->MoveRight(-50);

		if (Robot::driveSystem->GetContinuousYaw() >= -60) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 2;
		}
		break;
	case 2:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 2.56) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 3;
			//reset Sensors
		}
		break;
	case 3:
		Robot::driveSystem->MoveLeft(-50);
		Robot::driveSystem->MoveRight(50);

		if (Robot::driveSystem->GetContinuousYaw() <= 90) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 4;
		}
		break;
	case 4:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 1.31) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			isFinished = true;
			//reset Sensors
		}
		break;
	default:
		isFinished = true;
	}

	return isFinished;
}

bool AutonomousCommand::Auto5() {
	static bool isFinished = false;

	switch (autoStage) {
	case 0:
		Robot::driveSystem->ResetEncoders();
		Robot::driveSystem->ResetYaw();
		autoStage = 1;
		break;
	case 1:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
				>= 2.19) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 2;
			//reset Sensors
		}
		break;
	case 2:
		Robot::driveSystem->MoveLeft(-50);
		Robot::driveSystem->MoveRight(50);

		if (Robot::driveSystem->GetContinuousYaw() <= 30) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			autoStage = 3;
		}
		break;
	case 3:
		//Motor Stuff
		if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(45);
		} else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
			Robot::driveSystem->MoveLeft(45);
			Robot::driveSystem->MoveRight(50);
		} else {
			Robot::driveSystem->MoveLeft(50);
			Robot::driveSystem->MoveRight(50);
		}

		if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft) >= .82) {
			Robot::driveSystem->ResetEncoders();
			Robot::driveSystem->ResetYaw();
			isFinished = true;
		}
		break;
	default:
		isFinished = true;
	}

	return isFinished;
}
