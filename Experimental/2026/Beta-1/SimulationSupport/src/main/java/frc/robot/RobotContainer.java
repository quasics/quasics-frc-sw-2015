// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Direct imports, to provide simple example of crash under simulation.
// TODO: Remove when the bug is fixd.
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// Primary imports.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;

public class RobotContainer {
  private static final boolean DEMO_REV_CRASH = true;

  IDrivebase drivebase = DEMO_REV_CRASH ? null : new Drivebase();

  final SparkMax sampleController = DEMO_REV_CRASH ? new SparkMax(10, MotorType.kBrushless) : null;

  public RobotContainer() {
    if (drivebase != null) {
      drivebase.asSubsystem().setDefaultCommand(
          new ArcadeDrive(drivebase, this::getArcadeForward, this::getArcadeRotation));
    }

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Double getArcadeForward() {
    // TODO: Replace with real input
    return 0.0;
  }

  public Double getArcadeRotation() {
    // TODO: Replace with real input
    return 0.0;
  }
}