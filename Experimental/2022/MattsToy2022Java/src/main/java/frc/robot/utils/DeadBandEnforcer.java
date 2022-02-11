// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Provides support for "deadband" handling".
 * 
 * @see https://en.wikipedia.org/wiki/Deadband
 */
public class DeadBandEnforcer implements SpeedModifier {
    /** The lower bound on the dead band. */
    private final double lowBar;

    /** The upper bound on the dead band. */
    private final double highBar;

    /**
     * Sets up a dead band configured from "-threshold" to "+threshold".
     * 
     * @param threshold defines upper/lower bound on the dead band around 0
     */
    public DeadBandEnforcer(double threshold) {
        this(-Math.abs(threshold), Math.abs(threshold));
    }

    /**
     * Sets up a dead band configured from "-lowBar" to "+highBar".
     * 
     * @param lowBar  defines lower bound on the dead band
     * @param highBar defines upper bound on the dead band
     */
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
    @Override
    public double adjustSpeed(double value) {
        if (value > lowBar && value < highBar) {
            return 0;
        }
        return value;
    }
}
