// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IClimber;

public class RealClimber extends SubsystemBase implements IClimber {
  static final String DIRECTION_PREFS_KEY = "ClimberDirection";
  static final String NORMAL_DIRECTON_LABEL = "Normal";
  static final int NORMAL_DIRECTION_VALUE = +1;
  static final String INVERTED_DIRECTON_LABEL = "Inverted";
  static final int INVERTED_DIRECTION_VALUE = -1;

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

    // Try to load the configured direction from saved preferences (defaulting to
    // +1).
    m_configuredDirection = Preferences.getInt(DIRECTION_PREFS_KEY, +1);

    m_chooser.addOption(NORMAL_DIRECTON_LABEL, NORMAL_DIRECTION_VALUE);
    m_chooser.addOption(INVERTED_DIRECTON_LABEL, INVERTED_DIRECTION_VALUE);
    if (m_configuredDirection > 0) {
      m_chooser.setDefaultOption("Normal", NORMAL_DIRECTION_VALUE);
    } else {
      m_chooser.setDefaultOption("Inverted", INVERTED_DIRECTION_VALUE);
    }

    Shuffleboard.getTab("Climber").add("Direction", m_chooser);
    m_chooser.onChange(this::directionSelectionChanged);
  }

  private void directionSelectionChanged(Integer direction) {
    m_logger.log(Logger.Verbosity.Info, "Direction changed to " + direction);
    m_configuredDirection = direction;
    Preferences.setInt(DIRECTION_PREFS_KEY, m_configuredDirection);
  }

  @Override
  public void setClimberSpeed(double speed) {
    m_logger.logFormatted(Logger.Verbosity.Info, "Setting climber speed to %0.2f (%s)", speed,
        m_configuredDirection > 0 ? "Normal" : "Inverted");
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
