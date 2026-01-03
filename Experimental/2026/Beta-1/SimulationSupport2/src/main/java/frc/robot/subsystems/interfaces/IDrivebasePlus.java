package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IDrivebasePlus extends IDrivebase {
  public Pose2d getEstimatedPose();
  public DifferentialDriveWheelSpeeds getWheelSpeeds();
  public void tankDriveVolts(Voltage leftVoltage, Voltage rightVoltage);

  void setSpeeds(ChassisSpeeds speeds);

  Distance getLeftPosition();
  LinearVelocity getLeftVelocity();
  Voltage getLeftVoltage();
  Distance getRightPosition();
  LinearVelocity getRightVelocity();
  Voltage getRightVoltage();

  /** Returns the static gain for the drivebase (generally computed using the SysID tool). */
  double getKs();

  /** Returns the velocity gain for the drivebase (generally computed using the SysID tool). */
  double getKv();

  /** Returns the acceleration gain for the drivebase (generally computed using the SysID tool). */
  double getKa();

  /**
   * Returns the kP value for the drivebase to convert velocity errors (in m/s) to voltages
   * (generally computed using the SysID tool).
   */
  double getKp();
}
