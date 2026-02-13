// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import edu.wpi.first.units.measure.Angle;

/**
 * Describes a camera's orientiation relative to the robot.
 *
 * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
 * @param roll  The counterclockwise rotation angle around the X axis (roll).
 * @param yaw   The counterclockwise rotation angle around the Z axis (yaw).
 */
public record Orientation(Angle pitch, Angle roll, Angle yaw) {
}