// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IShooterHood;

public class RealShooterHood extends SubsystemBase implements IShooterHood {

  private final SparkMax m_hood;
  private final AbsoluteEncoder m_throughBoreEncoder;
  private final Logger m_logger = new Logger(Verbosity.Info, "RealShooterHood");

  /** Creates a new RealShooterHood. */
  public RealShooterHood() {

    m_hood = new SparkMax(SparkMaxIds.HOOD_ID, MotorType.kBrushless);

    m_throughBoreEncoder = m_hood.getAbsoluteEncoder();

  }

  @Override
  public double getCurrentAngle() {

    double m_currentAngleDegrees = m_throughBoreEncoder.getPosition() * 360;

    if (m_currentAngleDegrees > 300) {

      m_currentAngleDegrees = 0;

    }

    return m_currentAngleDegrees;

  }

  @Override
  public void moveDown(double speed) {

    m_hood.set(Math.abs(speed));

  }

  @Override
  public void moveUp(double speed) {

    m_hood.set(-Math.abs(speed));

  }

  @Override
  public void stop() {

    m_hood.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_logger.log("Current Angle: " + getCurrentAngle(), Verbosity.Debug);
  }

}
