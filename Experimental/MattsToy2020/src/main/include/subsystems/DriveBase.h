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

#include "utils/ShuffleboardWrappers.h"

/**
 * Sample code for a drive base, written to operate in "tank drive"
 * configuration.
 */
class DriveBase : public frc2::SubsystemBase {
 public:
  /**
   * Constructor.  Drive base will be set up with the motors configured for
   * "hard stop", and the encoders reset to 0 values.
   */
  DriveBase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.  (Used to
   * update the view of the encoders on the Shuffleboard.)
   *
   * @see #ReportEncoderDataToShuffleboard()
   */
  void Periodic();

  ////////////////////////////////////
  //
  // Basic motor control
  //
  ////////////////////////////////////
 public:
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
  void EnableTurboMode();

  /** Disables "turbo" mode.  (Will take effect the next time thst
   * SetMotorPower() is invoked.) */
  void DisableTurboMode();

  /**
   * Establishes if the motors should coast to a stop (true) or come to a hard
   * stop (false).
   */
  void SetCoastingEnabled(bool enabled);

  /**
   * Returns true iff the motors are currently configured to coast to a stop.
   */
  bool IsCoastingEnabled();

  ////////////////////////////////////
  //
  // Encoder support
  //
  ////////////////////////////////////
 public:
  /**
   * Enumerates individual motors and sets of motors, for use with encoder
   * functions.
   */
  enum class Motors {
    LeftFront = 0b00001,   // 0x001,
    LeftRear = 0b00010,    // 0x002,
    LeftAll = 0b00011,     // 0x003,
    RightFront = 0b00100,  // 0x004,
    RightRear = 0b01000,   // 0x008,
    RightAll = 0b01101,    // 0x00C,
    All = 0b01111,         // 0x00F
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

  ////////////////////////////////////
  //
  // Debugging support
  //
  ////////////////////////////////////
 public:
  /**
   * Utility function (for debugging): display current encoder data to the smart
   * dashboard.
   */
  void ReportEncoderDataToShuffleboard();

  ////////////////////////////
  // Private utility functions
 private:
  /** Returns a pointer to the specified (single) encoder, or nullptr. */
  rev::CANEncoder* GetEncoder(Motors motor);

 private:
  ////////////////////////////////////
  //
  // Motor/encoder support
  //
  ////////////////////////////////////

  rev::CANSparkMax leftFront;
  rev::CANSparkMax leftRear;
  rev::CANSparkMax rightFront;
  rev::CANSparkMax rightRear;

  // Encoders (associated with the motors), yielding position/velocity data.
  rev::CANEncoder leftFrontEncoder = leftFront.GetEncoder();
  rev::CANEncoder leftRearEncoder = leftRear.GetEncoder();
  rev::CANEncoder rightFrontEncoder = rightFront.GetEncoder();
  rev::CANEncoder rightRearEncoder = rightRear.GetEncoder();

  // Functors used to scale/limit motor power.
  static const std::function<double(double)> joystickRangeLimiter;
  static const std::function<double(double)> standardPowerAdjuster;
  static const std::function<double(double)> turboPowerAdjuster;

  // Current limiter on motor power
  std::function<double(double)> powerAdjuster = standardPowerAdjuster;

  ////////////////////////////////////
  //
  // Shuffleboard/OI stuff
  //
  ////////////////////////////////////

  // Control on the Shuffleboard UI to turn debugging output on/off.  (Default
  // is off.)
  ShuffleboardWrappers::BooleanToggle loggingOn{false /* default value */,
                                                "DriveBase noisy" /* title */,
                                                "Logging" /* tab name */};

  ShuffleboardWrappers::Collection encodersList{"Encoders", "Drive base"};

  ShuffleboardWrappers::SimpleDisplay leftFrontEncoderTicksDisplay{
      "L. front (tx)", encodersList};
  ShuffleboardWrappers::SimpleDisplay leftRearEncoderTicksDisplay{
      "L. rear (tx)", encodersList};
  ShuffleboardWrappers::SimpleDisplay leftFrontEncoderInchesDisplay{
      "L. front (in)", encodersList};
  ShuffleboardWrappers::SimpleDisplay leftRearEncoderInchesDisplay{
      "L. rear (in)", encodersList};

  ShuffleboardWrappers::SimpleDisplay rightFrontEncoderTicksDisplay{
      "R. front (tx)", encodersList};
  ShuffleboardWrappers::SimpleDisplay rightRearEncoderTicksDisplay{
      "R. rear (tx)", encodersList};
  ShuffleboardWrappers::SimpleDisplay rightFrontEncoderInchesDisplay{
      "R. front (in)", encodersList};
  ShuffleboardWrappers::SimpleDisplay rightRearEncoderInchesDisplay{
      "R. rear (in)", encodersList};
};
