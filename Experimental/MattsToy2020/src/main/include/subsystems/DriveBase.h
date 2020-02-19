/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Quasics Robotics and Matthew J. Healy                   */
/* All Rights Reserved.                                                       */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <functional>
#include <iostream>

/**
 * Sample code for a drive base, written to operate in "tank drive"
 * configuration.
 */
class DriveBase : public frc2::SubsystemBase {
 public:
  /** Constructor. */
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  /**
   * Sets the power to be applied to the drive motors.
   *
   * @param leftPower power setting for the left-side motors (-1.0 to +1.0)
   * @param rightPower power setting for the left-side motors (-1.0 to +1.0)
   */
  void SetMotorPower(double leftPower, double rightPower);

  /** Stops the motors. */
  void Stop() {
    SetMotorPower(0, 0);
  }

  /** Enables "turbo" mode, where we increase maximum speed.  (Will take effect
   * the next time thst SetMotorPower() is invoked.) */
  void EnableTurboMode() {
    std::cout << "Enabling turbo mode" << std::endl;
    powerAdjuster = turboPowerAdjuster;
  }
  /** Disables "turbo" mode.  (Will take effect the next time thst
   * SetMotorPower() is invoked.) */
  void DisableTurboMode() {
    std::cout << "Disabling turbo mode" << std::endl;
    powerAdjuster = standardPowerAdjuster;
  }

  enum class Motors {
    LeftFront = 0x001,
    LeftRear = 0x002,
    LeftAll = 0x003,
    RightFront = 0x004,
    RightRear = 0x008,
    RightAll = 0x00C,
    All = 0x00F
  };

  /**
   * Returns the current position reported by the encoder on the specified
   * motor; if a motor group is specified, then the average across the group is
   * returned.
   *
   * @param motor the targeted motor (or motor group)
   *
   * @return encoder position (reported in "ticks", which should usually must be
   * converted to a more meaningful unit of measurement)
   *
   * @see #ConvertTicksToInches
   */
  double GetEncoderPosition(Motors motor);

  /**
   * Resets the encoder position on the specified motor (or motor group).
   *
   * @param motor the targeted motor (or motor group)
   */
  void ResetEncoderPosition(Motors motor);

  /**
   * Utility function (for debugging): display current encoder data to the smart
   * dashboard.
   */
  void ReportEncoderDataToSmartDashboard();

 private:
  /** Returns a pointer to the specified (single) encoder, or nullptr. */
  rev::CANEncoder* GetEncoder(Motors motor);

  /** Reports data for one encoder on the smart dashboard. */
  void ReportEncoderDataToSmartDashboard(std::string prefix,
                                         rev::CANEncoder& encoder);

 private:
  // Motors
  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

  // Encoders (associated with the motors), yielding position/velocity data
  rev::CANEncoder leftFrontEncoder = leftFront.GetEncoder();
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();

  // Fucntors used to scale/limit motor power.
  static const std::function<double(double)> joystickRangeLimiter;
  static const std::function<double(double)> standardPowerAdjuster;
  static const std::function<double(double)> turboPowerAdjuster;

  // Current limiter on motor power
  std::function<double(double)> powerAdjuster = turboPowerAdjuster;
};
