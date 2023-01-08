// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public abstract class ThreeAxisGyro {
    public final class NullGyro implements Gyro {
        // From Gyro
        public void calibrate() {}
        public double getAngle() { return 0; }
        public double getRate() { return 0; }
        public Rotation2d getRotation2d() { return new Rotation2d(); }
        public void reset() {}

        // From AutoCloseable
        public void close() {}
    }

    public abstract Gyro getXAxisGyro();
    public abstract Gyro getYAxisGyro();
    public abstract Gyro getZAxisGyro();
}
