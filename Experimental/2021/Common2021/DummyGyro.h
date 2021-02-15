#pragma once

#include <frc/interfaces/Gyro.h>

/**
 * A completely faked implementation of frc::Gyro, for use in satisfying the
 * requirements of the CommonDriveSubsystem interface, while we try to get
 * a real gyroscope/IMU wired up to the drive bases for Mae and Nike.
 */
class DummyGyro : public frc::Gyro {
 public:
  DummyGyro() = default;

  //
  // Abstract methods defined in frc::Gyro.
 public:
  void Reset() override {}
  double GetAngle() const override { return 0; }
  double GetRate() const override { return 0; }
  void Calibrate() override {}
};
