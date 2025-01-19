// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveForDistance;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.simulations.SimDrivebase;

public class RobotContainer {
  // Subsystems
  final Vision m_vision = new Vision.SimulatedVision();
  private final IDrivebase m_drivebase = new SimDrivebase();

  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  public RobotContainer() {
    configureArcadeDrive();
    configureBindings();
  }

  private void configureArcadeDrive() {
    Supplier<Double> arcadeDriveForwardStick;
    Supplier<Double> arcadeDriveRotationStick;

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      arcadeDriveForwardStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.LeftYAxis);
      arcadeDriveRotationStick = () -> -m_driveController
          .getRawAxis(Constants.LogitechGamePad.RightXAxis);
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 1&2.
      arcadeDriveForwardStick = () -> -m_driveController.getRawAxis(0);
      arcadeDriveRotationStick = () -> -m_driveController.getRawAxis(1);
    }

    m_drivebase.asSubsystem().setDefaultCommand(new ArcadeDrive(m_drivebase, arcadeDriveForwardStick,
        arcadeDriveRotationStick));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new DriveForDistance(m_drivebase, .50, Meters.of(3));
    // return Commands.print("No autonomous command configured");
  }
}
