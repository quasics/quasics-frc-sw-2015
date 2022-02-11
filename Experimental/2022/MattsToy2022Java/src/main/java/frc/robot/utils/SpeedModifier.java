// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * General interface for things that will manipulate motor speeds specified by
 * the driver joysticks.
 */
public interface SpeedModifier {
    double adjustSpeed(double inputPercentage);
}
