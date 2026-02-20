// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private double m_ff;
  private final VoltageOut m_sysIdControl = new VoltageOut(0);

  /** Creates a new RealShooter. */
  public RealShooter() {
    m_kraken = new TalonFX(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    m_kraken.getConfigurator().apply(config);
    var slot0Configs = new Slot0Configs();
    // TODO: tune these
    slot0Configs.kV = 0.0;
    slot0Configs.kP = 0.0;
    m_ff = 0.0;
    m_kraken.getConfigurator().apply(slot0Configs);
    m_request = new VelocityVoltage(0).withSlot(0);
    m_kicker = new SparkMax(SparkMaxIds.KICKER_ID, MotorType.kBrushless);
    SignalLogger.start();
  }

  @Override
  public void setFlywheelVoltage(double rps) {
    m_kraken.setControl(m_request.withVelocity(rps).withFeedForward(m_ff));
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

  public double getFlywheelSpeed() {
    double speed = m_kraken.get();
    return speed;
  }

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(volts -> m_kraken.setControl(m_sysIdControl.withOutput(volts)), null, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel Speed: ", getFlywheelSpeed());
  }
}
