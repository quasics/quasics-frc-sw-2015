// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import edu.wpi.first.units.measure.Distance;

/**
 * Location of something in terms of the robot's center (in terms of the robot
 * coordinate system).
 *
 * This is currently used for the camera(s), but could easily be used for
 * other things as needed. (Note: I could've used a Translation3D for this,
 * but felt that "position" was more readable.)
 *
 * @param x distance along the X axis (front (+)/back (-)) from center
 * @param y distance along the Y axis (left (+)/right (-)) from center
 * @param z distance along the Z axis (up (+)/down (-)) from center
 *
 * @see <a
 *      href=
 *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system">Robot
 *      coordinate system</a>
 */
public record Position(Distance x, Distance y, Distance z) {}