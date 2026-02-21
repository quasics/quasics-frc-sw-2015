// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import edu.wpi.first.units.measure.Angle;

/**
 * Defines the image-related characteristics of a camera on the robot.
 *
 * Note that some of these characteristics would only be used (directly) in
 * the code for simulation purposes.
 *
 * @param width  camera field width (in pixels)
 * @param height camera field height (in pixels)
 * @param fov    field of view (e.g., 100 degrees)
 * @param fps    frames per second produced by the video stream
 */
public record Imaging(int width, int height, Angle fov, double fps) {}