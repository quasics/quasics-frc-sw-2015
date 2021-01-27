#pragma once

#include <frc/interfaces/Gyro.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

/**
 * Defines an interface for a drive base subsystem that could be implemented
 * for either a Romi device or a "real" FRC bot.
 * 
 * TODO: Consider other possibilities that the team likes, such as "flip mode",
 * "turbo/turtle mode", etc.
 */
class CommonDriveSubsystem : public frc2::SubsystemBase {
 public:
  CommonDriveSubsystem() = default;

  /**
   * Stops the drive motors.
   */
  virtual void Stop() = 0;

  /**
   * Drives the robot using arcade controls.
   *
   * @param xaxisSpeed the commanded forward movement
   * @param zaxisRotate the commanded rotation
   * @param squareInputs if set, increases the sensitivity at low speeds
   */
  virtual void ArcadeDrive(double xaxisSpeed, double zaxisRotate,
                           double squareInputs = true) = 0;

  /**
   * Drives the robot using tank drive (left and right powered independently).
   *
   * @param leftSpeed the commanded speed for the left side
   * @param rightSpeed the commanded speed for the right side
   */
  virtual void TankDrive(double leftSpeed, double rightSpeed) = 0;

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  virtual void ResetEncoders() = 0;

  /**
   * Gets the left drive encoder count.  This may be "ticks" or "rotations"
   * (or something else), depending on the underlying encoder.
   *
   * @return the left drive encoder count
   */
  virtual double GetLeftEncoderCount() = 0;

  /**
   * Gets the right drive encoder count.  This may be "ticks" or "rotations"
   * (or something else), depending on the underlying encoder.
   *
   * @return the right drive encoder count
   */
  virtual double GetRightEncoderCount() = 0;

  /**
   * Gets the left distance driven.
   *
   * @return the left-side distance driven
   */
  virtual units::meter_t GetLeftDistance() = 0;

  /**
   * Gets the right distance driven.
   *
   * @return the right-side distance driven
   */
  virtual units::meter_t GetRightDistance() = 0;

  /**
   * Returns the average distance traveled by the left and right encoders.
   *
   * @return The average distance traveled by the left and right encoders.
   */
  virtual units::meter_t GetAverageDistance() {
    // Baseline implementation.
    return (GetLeftDistance() + GetRightDistance()) / 2;
  }

  /**
   * Reset the gyro.
   */
  virtual void ResetGyro() {
    // Baseline implementation.
    GetZAxisGyro().Reset();
  }

  /**
   * Provides access to the gyro responsible for tracking Z-axis rotation.
   */
  virtual frc::Gyro& GetZAxisGyro() = 0;
};
