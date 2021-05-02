// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class SpeedScaler {
    public enum Mode {
        NORMAL, TURBO, TURTLE
    }

    public interface ModeFunction {
        public Mode get();
    }

    final ModeFunction modeFunction;
    final double normal;
    final double turbo;
    final double turtle;

    public SpeedScaler(ModeFunction modeFunction, double normal, double turbo, double turtle) {
        this.modeFunction = modeFunction;
        this.turbo = Math.max(Math.max(normal, turtle), turbo);
        this.turtle = Math.min(Math.min(normal, turtle), turbo);
        this.normal = middleOfThree(normal, turtle, turbo);
    }

    public PowerFunction apply(PowerFunction powerFunction) {
        return new PowerFunction() {
            @Override
            public double get() {
                double factor = normal;
                switch (modeFunction.get()) {
                    case NORMAL:
                        factor = normal;
                        break;
                    case TURBO:
                        factor = turbo;
                        break;
                    case TURTLE:
                        factor = turtle;
                        break;
                }
                return powerFunction.get() * factor;
            }
        };
    }

    /** Function to find the middle of three numbers. */
    public static double middleOfThree(double a, double b, double c) {
        // x is positive if a is greater than b.
        // x is negative if b is greater than a.
        double x = a - b;

        double y = b - c; // Similar to x
        double z = a - c; // Similar to x and y.

        // Checking if b is middle (x and y both are positive)
        if (x * y > 0)
            return b;

        // Checking if c is middle (x and z both are positive)
        else if (x * z > 0)
            return c;
        else
            return a;
    }
}
