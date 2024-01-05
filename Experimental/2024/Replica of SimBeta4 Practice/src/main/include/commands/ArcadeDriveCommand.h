#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IDrivebase.h"

class ArcadeDriveCommand
    : public frc2::CommandHelper<frc2::Command, ArcadeDriveCommand> {
 public:
  typedef std::function<double()> PercentSupplier;

 private:
  IDrivebase& m_drivebase;
  PercentSupplier m_forwardSupplier;
  PercentSupplier m_rotationSupplier;

//so joysticks are not as harsh
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

 public:
  ArcadeDriveCommand(IDrivebase& drivebase, PercentSupplier forwardSupplier,
                     PercentSupplier rotationSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  void updateSpeeds();
};

