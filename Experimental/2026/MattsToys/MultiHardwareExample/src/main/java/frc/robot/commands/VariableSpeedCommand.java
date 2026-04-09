// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleMotorThing;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VariableSpeedCommand extends Command {
  final GenericEntry m_speedSlider;
  final GenericEntry m_speedDisplay;
  final ISingleMotorThing m_subsystem;

  static public final String TAB_NAME = "Variable (%) Speed";
  static final String NOT_RUNNING_STRING = "---";

  /**
   * Constructor.
   */
  public VariableSpeedCommand(ISingleMotorThing subsystem) {
    m_subsystem = subsystem;
    m_speedSlider = Shuffleboard.getTab(TAB_NAME)
        .add("Motor Percent", 0.0) // Label and Default Value
        .withWidget(BuiltInWidgets.kNumberSlider) // Tell it to be a slider
        .withProperties(Map.of("min", -1.0, "max", +1.0)) // Set slider range
        .getEntry();
    m_speedDisplay = Shuffleboard.getTab(TAB_NAME)
        .add("Active: ", NOT_RUNNING_STRING)
        .withWidget(BuiltInWidgets.kTextView) // Standard text display
        .getEntry();
    addRequirements((SubsystemBase) m_subsystem);
  }

  @Override
  public void execute() {
    double targetValue = MathUtil.clamp(m_speedSlider.getDouble(0.0), -1.0, +1.0);
    m_speedDisplay.setString(String.format("%.2f", targetValue));
    m_speedSlider.setDouble(targetValue);
    m_subsystem.setSpeed(targetValue);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    m_speedDisplay.setString(NOT_RUNNING_STRING);
  }
}
