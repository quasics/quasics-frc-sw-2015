// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import java.util.Collections;
import java.util.List;

/**
 * Collective robot configuration data.
 *
 * @param drive    drive base configuration (may be null)
 * @param cameras  list of camera configurations (may be null/empty)
 * @param elevator elevator configuration (may be null)
 * @param lighting lighting configuration (may be null)
 * @param arm      arm configuration (may be null)
 * @param candle   CANdle configuration (may be null)
 * @param climber  climber configuration (may be null)
 * @param flywheel flywheel configuration (may be null)
 * @param hood     hood configuration (may be null)
 * @param power    power distribution configuration (may be null)
 */
public record RobotConfig(
    boolean isSimulated,
    DriveConfig drive,
    List<CameraConfig> cameras,
    ElevatorConfig elevator,
    ArmConfig arm,
    LightingConfig lighting,
    CandleConfig candle,
    ClimberConfig climber,
    FlywheelConfig flywheel,
    HoodConfig hood,
    PowerDistributor power) {
  public RobotConfig(
      boolean isSimulated,
      DriveConfig drive,
      List<CameraConfig> cameras,
      ElevatorConfig elevator,
      ArmConfig arm,
      LightingConfig lighting,
      CandleConfig candle,
      ClimberConfig climber,
      FlywheelConfig flywheel,
      HoodConfig hood,
      PowerDistributor power) {
    if (drive != null) {
      assert (isSimulated == (drive.driveType() == DriveType.Simulated))
          : "Simulation setting mismatch (robot vs. drive)";
      if (isSimulated != (drive.driveType() == DriveType.Simulated)) {
        throw new RuntimeException(
            "Simation setting mismatch (runtime vs. config)");
      }
    }
    this.isSimulated = isSimulated;
    this.drive = drive;
    this.cameras = cameras;
    this.elevator = elevator;
    this.arm = arm;
    this.lighting = lighting;
    this.candle = candle;
    this.climber = climber;
    this.flywheel = flywheel;
    this.hood = hood;
    this.power = power;
  }

  /**
   * Utility constructor fo a single-camera robot.
   *
   * @param drive    drive base configuration (may be null)
   * @param camera   camera configuration (may be null)
   * @param elevator elevator configuration (may be null)
   * @param lighting lighting configuration (may be null)
   * @param arm      arm configuration (may be null)
   * @param candle   CANdle configuration (may be null)
   * @param climber  climber configuration (may be null)
   * @param flywheel flywheel configuration (may be null)
   * @param hood     hood configuration (may be null)
   * @param power    power distribution configuration (may be null)
   */
  public RobotConfig(
      boolean isSimulated,
      DriveConfig drive,
      CameraConfig camera,
      ElevatorConfig elevator,
      ArmConfig arm,
      LightingConfig lighting,
      CandleConfig candle,
      ClimberConfig climber,
      FlywheelConfig flywheel,
      HoodConfig hood,
      PowerDistributor power) {
    this(isSimulated, drive,
        camera != null ? Collections.singletonList(camera) : null, elevator,
        arm, lighting, candle, climber, flywheel, hood, power);
  }

  /**
   * Determines if we have power distribution configuration data.
   *
   * @return true iff the configuration includes data for the power
   *         distribution
   *         setup
   */
  public boolean hasPowerDistributor() {
    return power != null;
  }

  /**
   * Determines if we have drive configuration data.
   *
   * @return true iff the configuration includes data for the drivebase
   */
  public boolean hasDrive() {
    return drive != null;
  }

  /**
   * Determines if we have camera configuration data.
   *
   * @return true iff the configuration includes data for the camera
   */
  public boolean hasCamera() {
    return (cameras() != null) && !cameras.isEmpty();
  }

  /**
   * Determines if we have elevator configuration data.
   *
   * @return true iff the configuration includes data for the elevator
   */
  public boolean hasElevator() {
    return elevator != null;
  }

  /**
   * Determines if we have arm configuration data.
   *
   * @return true iff the configuration includes data for the arm
   */
  public boolean hasArm() {
    return arm != null;
  }

  /**
   * Determines if we have lighting configuration data.
   *
   * @return true iff the configuration includes data for the lighting
   */
  public boolean hasLighting() {
    return lighting != null;
  }

  /**
   * Determines if we have CANdle configuration data.
   *
   * @return true iff the configuration includes data for the CANdle
   */
  public boolean hasCandle() {
    return candle != null;
  }

  /**
   * Determines if we have climber configuration data.
   *
   * @return true iff the configuration includes data for the climber
   */
  public boolean hasClimber() {
    return climber != null;
  }

  /**
   * Determines if we have flywheel configuration data.
   *
   * @return true iff the configuration includes data for the flywheel
   */
  public boolean hasFlywheel() {
    return flywheel != null;
  }

  /**
   * Determines if we have hood configuration data.
   *
   * @return true iff the configuration includes data for the hood
   */
  public boolean hasHood() {
    return hood != null;
  }
}