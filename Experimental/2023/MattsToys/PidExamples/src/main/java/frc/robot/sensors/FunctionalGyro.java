package frc.robot.sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class FunctionalGyro implements Gyro {

  final Runnable m_close;
  final Runnable m_calibrate;
  final Runnable m_reset;
  final DoubleSupplier m_getAngle;
  final DoubleSupplier m_getRate;

  public FunctionalGyro(
      Runnable close,
      final Runnable calibrate,
      final Runnable reset,
      final DoubleSupplier getAngle,
      final DoubleSupplier getRate) {
    m_close = close;
    m_calibrate = calibrate;
    m_reset = reset;
    m_getAngle = getAngle;
    m_getRate = getRate;
  }

  @Override
  public void close() throws Exception {
    m_close.run();
  }

  @Override
  public void calibrate() {
    m_calibrate.run();
  }

  @Override
  public void reset() {
    m_reset.run();
  }

  @Override
  public double getAngle() {
    return m_getAngle.getAsDouble();
  }

  @Override
  public double getRate() {
    return m_getRate.getAsDouble();
  }
}
