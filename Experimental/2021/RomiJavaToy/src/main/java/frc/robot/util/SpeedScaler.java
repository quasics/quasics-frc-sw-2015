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
        this.normal = normal;
        this.turbo = turbo;
        this.turtle = turtle;
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
}
