// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface RobotSettings {
  /**
   * Different possible configerations for motors on the drivebase.
   */
  public enum MotorConfigModel {
    // No leader: Every motor needs driven seperately
    NoLeader,
    // Rear motors are configured as leader (with CAN)
    RearMotorsLeading,
    // front motors are configured as leader (with CAN)
    FrontMotorsLeading,
  }

  /**
   * Enum class represents characteristics (track width,
   * motor configuration, etc.) specific to a given robot
   */
  public enum Robot {
    /**
     * TODO: add simulator, margeret, etc. robot values
     * TODO: characterization values, robot to camera, TBD other values
     */
    Simulator(MotorConfigModel.RearMotorsLeading, Meter.of(0.381 * 2), 1.0),
    Sally(
        // Motor config model
        MotorConfigModel.RearMotorsLeading,
        // track width in meters
        Meters.of(0.5588) /* 22 in */,
        // gear ratio
        8.45),
    NewRobot(MotorConfigModel.RearMotorsLeading, Meters.of(0.5628), /*22 5/32 in */
        0);

    ////////////////////////////////////////////////
    // Drivebase data
    public final MotorConfigModel motorConfigModel;
    public final Distance trackWidthMeters;
    public final double gearRatio;

    private Robot(MotorConfigModel motorConfigModel, Distance trackWidthMeters, double gearRatio) {
      this.motorConfigModel = motorConfigModel;
      this.trackWidthMeters = trackWidthMeters;
      this.gearRatio = gearRatio;
    }
  }
}
