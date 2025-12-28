// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.util.DriverJoystickWrapper;

public class RobotContainer {
  private final IDrivebase drivebase = new Drivebase();
  private final DriverJoystickWrapper m_driverWrapper = new DriverJoystickWrapper(
      OperatorConstants.DRIVER_JOYSTICK_ID);
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  /** Constructor. */
  public RobotContainer() {
    drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(drivebase, this::getArcadeForward, this::getArcadeRotation));

    configureBindings();
  }

  private void configureBindings() {
    // Set up autonomous command chooser
    autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    SmartDashboard.putData("Autonomous Command", autoCommandChooser);
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (autoCommandChooser.getSelected() != null) {
      return autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }

  //
  // Drivebase control support
  //

  /** Slew rate limiter for forward control. */
  private final SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);

  /**
   * Returns the "forward" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeForward() {
    return forwardSlewRateLimiter.calculate(m_driverWrapper.getArcadeForward());
  }

  /** Slew rate limiter for rotation control. */
  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);

  /**
   * Returns the "rotation" value for arcade drive based on the current control
   * scheme.
   */
  public Double getArcadeRotation() {
    return rotationSlewRateLimiter.calculate(m_driverWrapper.getArcadeRotation());
  }
}