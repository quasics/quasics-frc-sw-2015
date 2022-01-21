// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Provides support for "deadband" handling.
 * 
 * @see https://en.wikipedia.org/wiki/Deadband
 */
public class DeadBandEnforcer {
    private final double lowBar;
    private final double highBar;

    public DeadBandEnforcer(double threshold) {
        this(-Math.abs(threshold), Math.abs(threshold));
    }

    public DeadBandEnforcer(double lowBar, double highBar) {
        this.lowBar = Math.min(lowBar, highBar);
        this.highBar = Math.max(lowBar, highBar);
    }

    /**
     * Applies the deadband definition to the specified value.
     * 
     * @param value the value to be normalized/restricted against the configured
     *              deadband
     * @return 0 if the value is between the low/high bar for the deadband, or else
     *         the input value otherwise
     */
    public double restrict(double value) {
        if (value > lowBar && value < highBar) {
            return 0;
        }
        return value;
    }
}
