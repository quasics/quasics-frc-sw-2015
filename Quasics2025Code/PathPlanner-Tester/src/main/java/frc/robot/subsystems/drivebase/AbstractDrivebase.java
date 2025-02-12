// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractDrivebase extends SubsystemBase {
  /** Creates a new AbstractDrivebase. */

  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);

  // Max rotational speed i 1/2 rotation per second (pi radians per second)
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(Math.PI);

  public static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final DifferentialDriveKinematics m_kinematics;

  private final Distance trackWidthMeters = Meters.of(0.5588);

  public AbstractDrivebase() {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
  }

  public final void stop() {}

  public final void arcadeDrive(LinearVelocity fSpeed, AngularVelocity rot) {
    fSpeed =
        fSpeed.gt(MAX_SPEED.unaryMinus()) ? fSpeed : MAX_SPEED.unaryMinus();
    fSpeed = fSpeed.lt(MAX_SPEED) ? fSpeed : MAX_SPEED;
    rot = rot.gt(MAX_ANGULAR_SPEED.unaryMinus())
              ? rot
              : MAX_ANGULAR_SPEED.unaryMinus();
    rot = rot.lt(MAX_ANGULAR_SPEED) ? rot : MAX_ANGULAR_SPEED;

    setSpeed(
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rot)));
  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds) {}

  public void setSpeed(double leftSpeed, double rightSpeed) {}

  protected abstract void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
