// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BangBangFlywheel;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  BangBangFlywheel m_flywheel = new BangBangFlywheel();
  Shooter m_shooter = new Shooter(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData(
        "Flywheel - 2000 RPM",
        new InstantCommand(
            () -> {
              m_flywheel.setSpeed(RevolutionsPerSecond.of(2000.0 / 60.0));
            },
            m_flywheel));
    SmartDashboard.putData(
        "Shooter - 3000 RPM",
        new InstantCommand(
            () -> {
              m_shooter.setRPM(3000);
            },
            m_shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
