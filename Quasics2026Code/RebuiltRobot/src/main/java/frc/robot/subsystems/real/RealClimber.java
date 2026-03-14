// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IClimber;

public class RealClimber extends SubsystemBase implements IClimber {

  private SparkMax m_climber;
  private final Logger m_logger = new Logger(Verbosity.Info, "RealClimber");
  private final SendableChooser<Integer> m_chooser = new SendableChooser<>();
  private int m_configuredDirection = +1;

  private RelativeEncoder getEncoderClimber() {
    return m_climber.getEncoder();
  }

  /** Creates a new RealClimber. */
  public RealClimber() {
    m_climber = new SparkMax(SparkMaxIds.CLIMBER_ID, MotorType.kBrushless);
    m_logger.log(Verbosity.Info,
        "CLIMBER: Creating climber on " + SparkMaxIds.CLIMBER_ID + " temp: " + m_climber.getMotorTemperature());

    m_chooser.addOption("Normal", +1);
    m_chooser.addOption("Inverted", -1);
    Shuffleboard.getTab("Climber").add("Direction", m_chooser);
    m_chooser.onChange(this::directionSelectionChanged);
  }

  private void directionSelectionChanged(Integer direction) {
    m_logger.log(Logger.Verbosity.Info, "Direction changed to " + direction);
    m_configuredDirection = direction;
  }

  @Override
  public void setClimberSpeed(double speed) {
    m_climber.set(speed * m_configuredDirection);
  }

  @Override
  public void stopClimber() {
    m_climber.set(0);
  }

  @Override
  public double getClimberPosition() {
    return getEncoderClimber().getPosition();
  }

  // @Override
  // public void periodic() {
  // ***************************************************************************************************************
  // // This method will be called once per scheduler run
  // System.out.print(SparkMaxIds.CLIMBER_ID);
  // }
}
