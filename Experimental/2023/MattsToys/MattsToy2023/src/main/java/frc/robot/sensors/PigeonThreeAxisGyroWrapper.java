// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import java.util.function.Supplier;

/** Add your docs here. */
public class PigeonThreeAxisGyroWrapper extends ThreeAxisGyro {
  enum Axis {
    ePitch,
    eRoll,
    eYaw
  }

  private class SingleAxisWrapper implements Gyro {
    double m_calibrationOffset = 0;
    final Supplier<Double> m_angleSupplier;

    SingleAxisWrapper(Axis axis) {
      m_angleSupplier = getAngleForAxis(axis);
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void calibrate() {}

    @Override
    public void reset() {
      m_calibrationOffset = m_angleSupplier.get();
    }

    @Override
    public double getAngle() {
      return m_angleSupplier.get() - m_calibrationOffset;
    }

    @Override
    public double getRate() {
      throw new UnsupportedOperationException("getRate() isn't supported for Pigeon2");
    }
  }

  private Supplier<Double> getAngleForAxis(Axis axis) {
    switch (axis) {
      case ePitch:
        return m_pigeon::getPitch;
      case eRoll:
        return m_pigeon::getRoll;
      case eYaw:
        return m_pigeon::getYaw;
      default:
        return () -> {
          return 0.0;
        };
    }
  }

  private Pigeon2 m_pigeon;

  public PigeonThreeAxisGyroWrapper(int deviceNumber) {
    m_pigeon = new Pigeon2(deviceNumber);
  }

  @Override
  public Gyro getRollGyro() {
    return new SingleAxisWrapper(Axis.eRoll);
  }

  @Override
  public Gyro getPitchGyro() {
    return new SingleAxisWrapper(Axis.ePitch);
  }

  @Override
  public Gyro getYawGyro() {
    return new SingleAxisWrapper(Axis.eYaw);
  }
}
