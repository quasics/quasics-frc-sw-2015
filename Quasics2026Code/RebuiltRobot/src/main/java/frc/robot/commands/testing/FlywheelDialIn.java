// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IShooter;

/**
 * Simple command to drive the flywheel at different speeds, to be used in
 * "dialing in" the shooter.
 */
public class FlywheelDialIn extends Command {
  private final IShooter m_shooter;
  private GenericEntry m_speedSlider;

  /** Creates a new FlywheelDialIn. */
  public FlywheelDialIn(IShooter shooter) {
    m_shooter = shooter;
    addRequirements((Subsystem) shooter);
    m_speedSlider = Shuffleboard.getTab("Shooter")
        .add("Flywheel Target Speed", 0.0) // Label and Default Value
        .withWidget(BuiltInWidgets.kNumberSlider) // Tell it to be a slider
        .withProperties(Map.of("min", 0, "max", 6000)) // Set slider range (0-100% or RPS)
        .getEntry();
  }

  private void update() {
    double targetRPS = m_speedSlider.getDouble(0.0);
    m_shooter.setFlywheelRPM(RevolutionsPerSecond.of(targetRPS * 60));
    m_shooter.setKickerSpeed(1000);
  }

  @Override
  public void execute() {
    update();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheel();
  }
}
