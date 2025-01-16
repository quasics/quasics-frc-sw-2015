// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
  public static final Distance kWheelRadiusMeters = Units.Meters.of(0.0508);
  public static final Distance kRobotTrackWidthMeters = Units.Meters.of(0.381 * 2);
  public static final int kEncoderResolutionTicksPerRevolution = -4096;
}
