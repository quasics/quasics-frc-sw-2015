#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/voltage.h>

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
                           bool squareInputs = true) = 0;

  /**
   * Drives the robot using "curvature drive" controls (a specialized version
   * of arcade drive).
   *
   * @param xaxisSpeed the robot's speed along the X axis [-1.0..1.0]. Forward
   *                   is positive.
   * @param zaxisRotate the robot's rotation rate around the Z axis [-1.0..1.0].
   *                    Clockwise is positive
   * @param isQuickTurn if set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers
   */
  virtual void CurvatureDrive(double xaxisSpeed, double zaxisRotate,
                              bool isQuickTurn) = 0;

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

  /**
   * Adds the specified sendable (generally a subsystem-specific test command)
   * to the shuffleboard tab used by this subsystem.
   *
   * @param label the label (key) to display with the Sendable on the tab
   * @param data  the command (or other Sendable) to be put on the tab
   */
  virtual void AddToShuffleboard(std::string_view label,
                                 wpi::Sendable* data) = 0;

  //
  // Additional functions to support trajectory-following.
 public:
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  virtual frc::Pose2d GetPose() = 0;

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  virtual frc::DifferentialDriveWheelSpeeds GetWheelSpeeds() = 0;

  /**
   * Resets the odometry (i.e., resets the encoders to 0, and stores the
   * specified pose as our current one).
   *
   * @param pose The pose to which to set the odometry.
   */
  virtual void ResetOdometry(frc::Pose2d pose) = 0;

  /**
   * Controls each side of the drive directly with a voltage.
   * (Positive == forward, negative == reverse.)
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  virtual void TankDriveVolts(units::volt_t left, units::volt_t right) = 0;

  /**
   * Returns the track width.
   */
  virtual units::meter_t GetTrackWidth() = 0;
};
