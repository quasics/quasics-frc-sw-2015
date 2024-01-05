#include "commands/ArcadeDriveCommand.h"

//this is identical to any standard command we would do last year except for a few key differences
//its Idrivebase because its a parent class and therefore all drivebases will be Idrivebase
//Precent Supplier is the same thing as std function thingy, but its under a different name 
//for readibility purposes

ArcadeDriveCommand::ArcadeDriveCommand(IDrivebase& drivebase,
                                       PercentSupplier forwardSupplier,
                                       PercentSupplier rotationSupplier)
    : m_drivebase(drivebase),
      m_forwardSupplier(forwardSupplier),
      m_rotationSupplier(rotationSupplier) {
  AddRequirements(&m_drivebase);
}

// Called when the command is initially scheduled.
void ArcadeDriveCommand::Initialize() {
  updateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDriveCommand::Execute() {
  updateSpeeds();
}

// Called once the command ends or is interrupted.
void ArcadeDriveCommand::End(bool interrupted) {
  m_drivebase.stop();
}

void ArcadeDriveCommand::updateSpeeds() {
  const double forwardValue = m_forwardSupplier();
  const double rotationValue = m_rotationSupplier();

  const units::meters_per_second_t xSpeed =
      -m_speedLimiter.Calculate(forwardValue) * IDrivebase::MAX_SPEED;

  const units::radians_per_second_t rot =
      -m_rotLimiter.Calculate(rotationValue) * IDrivebase::MAX_ANGULAR_SPEED;

  IDrivebase::logValue("Forward stick", forwardValue);
  IDrivebase::logValue("Rotation stick", rotationValue);

  m_drivebase.arcadeDrive(xSpeed, rot);
}