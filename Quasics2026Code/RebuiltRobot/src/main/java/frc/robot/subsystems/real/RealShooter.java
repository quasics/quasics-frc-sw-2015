// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.interfaces.IShooter;

/**
 * Subsystem for controlling the shooter mechanism, which is used to shoot balls
 * into the Hub.
 * 
 * FINDME(Rylie): Some suggested reading for this subsystem:
 * * Straightforward PID control for flywheels:
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
 */
public class RealShooter extends SubsystemBase implements IShooter {
  private TalonFX m_kraken;
  private final VelocityVoltage m_request;
  private final SparkMax m_kicker;

  /** Creates a new RealShooter. */
  public RealShooter() {
    m_kraken = new TalonFX(0);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.0; // tune
    slot0Configs.kP = 0.0; // tune
    m_kraken.getConfigurator().apply(slot0Configs);
    m_request = new VelocityVoltage(0).withSlot(0);
    m_kicker = new SparkMax(SparkMaxIds.KICKER_ID, MotorType.kBrushless);
  }

  @Override
  public void setFlywheelVoltage(double rps) {
    m_kraken.setControl(m_request.withVelocity(rps).withFeedForward(0));
  }

  @Override
  public void setKickerSpeed(double speed) {
    m_kicker.set(speed);
  }

  public void setFlywheelSpeed(double speed) {
    m_kraken.set(speed);
  }

  @Override
  public void stopFlywheel() {
    m_kraken.set(0);
  }

  @Override
  public void stopKicker() {
    m_kicker.set(0);
  }

  public void stop() {
    stopFlywheel();
    stopKicker();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
